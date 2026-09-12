"""
Unit tests for GNSS Quality and Integrity Assessment Engine.
"""
import math
import numpy as np
import pytest

from core.sensors.data_types import GnssFix
from core.gnss.integrity import (
    GnssQualityEstimator,
    GnssClassification,
    GnssIntegrityConfig,
    GnssIntegrityReport,
)


def create_sample_fix(
    timestamp_ns: int = 1_000_000_000,
    lat: float = 37.7749,
    lon: float = -122.4194,
    alt: float = 10.0,
    vel_ned: tuple[float, float, float] = (5.0, 0.0, 0.0),
    hacc: float = 2.0,
    vacc: float = 3.0,
    sacc: float = 0.1,
    sats: int = 14,
) -> GnssFix:
    """Helper to create a deterministic GnssFix."""
    return GnssFix(
        timestamp_ns=timestamp_ns,
        latitude_deg=lat,
        longitude_deg=lon,
        altitude_m=alt,
        velocity_ned_mps=vel_ned,
        horizontal_accuracy_m=hacc,
        vertical_accuracy_m=vacc,
        speed_accuracy_mps=sacc,
        satellite_count=sats,
    )


def test_estimator_initialization():
    """Test GnssQualityEstimator initialization and custom config."""
    est = GnssQualityEstimator()
    assert est.config.trusted_threshold == 0.70
    assert est.config.degraded_threshold == 0.35

    cfg = GnssIntegrityConfig(trusted_threshold=0.80)
    est_custom = GnssQualityEstimator(cfg)
    assert est_custom.config.trusted_threshold == 0.80


def test_nominal_high_quality_gnss():
    """Test that a nominal GNSS fix with low accuracy error and high sat count is TRUSTED."""
    est = GnssQualityEstimator()
    fix = create_sample_fix(hacc=1.5, vacc=2.0, sacc=0.05, sats=16)

    report = est.evaluate(fix)

    assert report.classification == GnssClassification.TRUSTED
    assert report.quality_score > 0.80
    assert report.position_accuracy_score > 0.90
    assert report.velocity_accuracy_score > 0.90
    assert report.satellite_score == 1.0
    assert report.rejection_reason == ""
    # Scaled covariance should be close to base
    assert report.scaled_pos_cov[0, 0] < 5.0


def test_degraded_accuracy_gnss():
    """Test that high horizontal/vertical error degrades classification."""
    est = GnssQualityEstimator()
    # High horizontal error (15m), poor speed accuracy (1.5 m/s), low sats (6)
    fix = create_sample_fix(hacc=15.0, vacc=25.0, sacc=1.5, sats=6)

    report = est.evaluate(fix)

    assert report.classification == GnssClassification.DEGRADED
    assert est.config.degraded_threshold <= report.quality_score < est.config.trusted_threshold
    # Scaled covariance should be inflated significantly (> min inflation)
    base_var = fix.horizontal_accuracy_m ** 2
    assert report.scaled_pos_cov[0, 0] > base_var * est.config.degraded_cov_scale_min


def test_satellite_count_rejection():
    """Test that satellite count < 4 results in immediate low quality / rejection."""
    est = GnssQualityEstimator()
    fix = create_sample_fix(sats=3)

    report = est.evaluate(fix)

    assert report.satellite_score == 0.0
    assert report.classification == GnssClassification.REJECTED


def test_innovation_gating_trusted_and_rejection():
    """Test innovation scoring and hard gating."""
    est = GnssQualityEstimator()
    fix = create_sample_fix()

    # Case 1: Small innovation within 1-sigma
    pos_innov_small = np.array([0.5, 0.2, -0.1])
    pos_cov_small = np.eye(3) * 4.0  # std = 2.0m -> Mahalanobis ~ 0.28
    report_small = est.evaluate(fix, pos_innovation=pos_innov_small, pos_innovation_cov=pos_cov_small)
    assert report_small.innovation_score > 0.85
    assert report_small.classification == GnssClassification.TRUSTED

    # Case 2: Massive innovation exceeding hard gate (> 6 sigma)
    pos_innov_huge = np.array([50.0, 50.0, 0.0])  # Mahalanobis ~ 35 sigma
    report_huge = est.evaluate(
        create_sample_fix(timestamp_ns=2_000_000_000),
        pos_innovation=pos_innov_huge,
        pos_innovation_cov=pos_cov_small
    )
    assert report_huge.classification == GnssClassification.REJECTED
    assert "hard gate" in report_huge.rejection_reason


def test_temporal_jump_rejection():
    """Test rejection of impossible position jumps between fixes."""
    est = GnssQualityEstimator()
    # Fix 1 at origin
    fix1 = create_sample_fix(timestamp_ns=1_000_000_000, lat=37.0, lon=-122.0)
    report1 = est.evaluate(fix1)
    assert report1.classification == GnssClassification.TRUSTED

    # Fix 2 100ms later with a 500-meter jump (5000 m/s implied speed!)
    fix2 = create_sample_fix(timestamp_ns=1_100_000_000, lat=37.005, lon=-122.0)
    report2 = est.evaluate(fix2)

    assert report2.classification == GnssClassification.REJECTED
    assert "Position jump rate" in report2.rejection_reason


def test_fail_safe_numerical_validity():
    """Test fail-safe handling of NaN, Inf, negative accuracy, non-monotonic time."""
    est = GnssQualityEstimator()

    # 1. NaN in latitude
    fix_nan = create_sample_fix(lat=float('nan'))
    rep_nan = est.evaluate(fix_nan)
    assert rep_nan.classification == GnssClassification.REJECTED
    assert "NaN or Inf" in rep_nan.rejection_reason

    # 2. Inf in horizontal accuracy
    fix_inf = create_sample_fix(hacc=float('inf'))
    rep_inf = est.evaluate(fix_inf)
    assert rep_inf.classification == GnssClassification.REJECTED

    # 3. Negative accuracy
    fix_neg = create_sample_fix(hacc=-5.0)
    rep_neg = est.evaluate(fix_neg)
    assert rep_neg.classification == GnssClassification.REJECTED

    # 4. Implausible velocity (Mach 2!)
    fix_supersonic = create_sample_fix(vel_ned=(700.0, 0.0, 0.0))
    rep_supersonic = est.evaluate(fix_supersonic)
    assert rep_supersonic.classification == GnssClassification.REJECTED
    assert "Implausible velocity" in rep_supersonic.rejection_reason


def test_covariance_scaling_proportionality():
    """Test that covariance scales inversely with quality score."""
    est = GnssQualityEstimator()

    fix_high = create_sample_fix(hacc=2.0, vacc=3.0, sacc=0.1, sats=14)
    rep_high = est.evaluate(fix_high)

    fix_med = create_sample_fix(timestamp_ns=2_000_000_000, hacc=10.0, vacc=15.0, sacc=1.0, sats=8)
    rep_med = est.evaluate(fix_med)

    fix_low = create_sample_fix(timestamp_ns=3_000_000_000, hacc=20.0, vacc=30.0, sacc=2.5, sats=5)
    rep_low = est.evaluate(fix_low)

    assert rep_high.quality_score > rep_med.quality_score > rep_low.quality_score
    assert rep_high.scaled_pos_cov[0, 0] < rep_med.scaled_pos_cov[0, 0] < rep_low.scaled_pos_cov[0, 0]
