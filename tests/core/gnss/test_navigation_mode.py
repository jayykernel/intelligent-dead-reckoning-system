"""
Unit tests for Navigation Mode State Machine.
"""
import numpy as np
import pytest

from core.gnss.integrity import GnssClassification, GnssIntegrityReport
from core.gnss.navigation_mode import (
    NavigationModeManager,
    NavigationMode,
    NavigationModeConfig,
)


def _make_report(
    quality: float,
    classification: GnssClassification,
) -> GnssIntegrityReport:
    """Helper to create a minimal integrity report."""
    return GnssIntegrityReport(
        quality_score=quality,
        classification=classification,
        position_accuracy_score=quality,
        velocity_accuracy_score=quality,
        satellite_score=quality,
        innovation_score=quality,
        temporal_score=quality,
        scaled_pos_cov=np.eye(3) * (1.0 / max(quality, 0.01)),
        scaled_vel_cov=np.eye(3) * (1.0 / max(quality, 0.01)),
    )


def test_mode_initial_state():
    """Test that mode manager starts in GNSS_FIXED."""
    mgr = NavigationModeManager()
    assert mgr.current_mode == NavigationMode.GNSS_FIXED
    assert not mgr.is_in_outage


def test_gnss_loss_transition_with_persistence():
    """Test correct transition from GNSS_FIXED to DR via persistence counters."""
    cfg = NavigationModeConfig(outage_persistence_count=2)
    mgr = NavigationModeManager(cfg)

    rejected = _make_report(0.0, GnssClassification.REJECTED)

    # First rejected fix: should stay in GNSS_FIXED (persistence not met)
    mode1 = mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    assert mode1 == NavigationMode.GNSS_FIXED

    # Second rejected fix: persistence met -> TRANSITION_TO_DR
    mode2 = mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)
    assert mode2 == NavigationMode.TRANSITION_TO_DR
    assert mgr.is_in_outage


def test_transition_to_dr_to_dead_reckoning():
    """Test TRANSITION_TO_DR advances to DEAD_RECKONING on next rejected fix."""
    cfg = NavigationModeConfig(outage_persistence_count=1)
    mgr = NavigationModeManager(cfg)
    rejected = _make_report(0.0, GnssClassification.REJECTED)

    # 1 rejected -> TRANSITION_TO_DR
    mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    assert mgr.current_mode == NavigationMode.TRANSITION_TO_DR

    # Another rejected -> DEAD_RECKONING
    mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)
    assert mgr.current_mode == NavigationMode.DEAD_RECKONING


def test_gnss_recovery_path():
    """Test full GNSS recovery path: DR -> TRANSITION_TO_GNSS -> RECOVERED -> GNSS_FIXED."""
    cfg = NavigationModeConfig(
        outage_persistence_count=1,
        recovery_persistence_count=2,
        recovery_ramp_steps=3,
    )
    mgr = NavigationModeManager(cfg)

    rejected = _make_report(0.0, GnssClassification.REJECTED)
    trusted = _make_report(0.90, GnssClassification.TRUSTED)

    # Enter outage
    mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)
    assert mgr.current_mode == NavigationMode.DEAD_RECKONING

    # Start recovery: 2 consecutive trusted fixes
    mgr.update_with_gnss(trusted, timestamp_ns=5_000_000_000)
    mgr.update_with_gnss(trusted, timestamp_ns=6_000_000_000)
    assert mgr.current_mode == NavigationMode.TRANSITION_TO_GNSS

    # Confirm stability: requires recovery_persistence_count + 1 = 3 total
    mgr.update_with_gnss(trusted, timestamp_ns=7_000_000_000)
    assert mgr.current_mode == NavigationMode.RECOVERED

    # Ramp recovery over 3 steps
    mgr.step_recovery()  # step 1
    assert mgr.current_mode == NavigationMode.RECOVERED
    mgr.step_recovery()  # step 2
    assert mgr.current_mode == NavigationMode.RECOVERED
    mgr.step_recovery()  # step 3 -> complete, transitions to GNSS_FIXED
    assert mgr.current_mode == NavigationMode.GNSS_FIXED
    assert not mgr.is_in_outage


def test_false_alarm_recovery_rejection():
    """Test that a rejected fix during recovery returns to DR."""
    cfg = NavigationModeConfig(outage_persistence_count=1, recovery_persistence_count=2)
    mgr = NavigationModeManager(cfg)

    rejected = _make_report(0.0, GnssClassification.REJECTED)
    trusted = _make_report(0.9, GnssClassification.TRUSTED)

    # Enter DR
    mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)
    assert mgr.current_mode == NavigationMode.DEAD_RECKONING

    # Start recovery
    mgr.update_with_gnss(trusted, timestamp_ns=5_000_000_000)
    mgr.update_with_gnss(trusted, timestamp_ns=6_000_000_000)
    assert mgr.current_mode == NavigationMode.TRANSITION_TO_GNSS

    # False alarm: REJECTED fix during recovery -> back to DR
    mgr.update_with_gnss(rejected, timestamp_ns=7_000_000_000)
    assert mgr.current_mode == NavigationMode.DEAD_RECKONING


def test_hybrid_degraded_transition():
    """Test transition to HYBRID_DEGRADED from GNSS_FIXED."""
    cfg = NavigationModeConfig(degraded_persistence_count=2)
    mgr = NavigationModeManager(cfg)

    degraded = _make_report(0.5, GnssClassification.DEGRADED)
    trusted = _make_report(0.9, GnssClassification.TRUSTED)

    # 1st degraded: stay in GNSS_FIXED
    mgr.update_with_gnss(degraded, timestamp_ns=1_000_000_000)
    assert mgr.current_mode == NavigationMode.GNSS_FIXED

    # 2nd degraded: transition to HYBRID
    mgr.update_with_gnss(degraded, timestamp_ns=2_000_000_000)
    assert mgr.current_mode == NavigationMode.HYBRID_DEGRADED

    # Trusted fixes restore to GNSS_FIXED
    for i in range(3):
        mgr.update_with_gnss(trusted, timestamp_ns=3_000_000_000 + i * 1_000_000_000)

    assert mgr.current_mode == NavigationMode.GNSS_FIXED


def test_recovery_gain_scale():
    """Test that recovery gain scales from 0.1 (transition) up to 1.0 (ramp complete)."""
    cfg = NavigationModeConfig(
        outage_persistence_count=1,
        recovery_persistence_count=2,
        recovery_ramp_steps=5,
    )
    mgr = NavigationModeManager(cfg)

    rejected = _make_report(0.0, GnssClassification.REJECTED)
    trusted = _make_report(0.9, GnssClassification.TRUSTED)

    # Enter DR
    mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)

    # During DR, gain should be 1.0 (no GNSS scaling relevant)
    assert mgr.recovery_gain_scale == 1.0

    # Start recovery
    mgr.update_with_gnss(trusted, timestamp_ns=5_000_000_000)
    mgr.update_with_gnss(trusted, timestamp_ns=6_000_000_000)
    assert mgr.current_mode == NavigationMode.TRANSITION_TO_GNSS
    assert mgr.recovery_gain_scale == 0.1  # Minimal gain in transition

    # Advance to RECOVERED
    mgr.update_with_gnss(trusted, timestamp_ns=7_000_000_000)
    assert mgr.current_mode == NavigationMode.RECOVERED

    # Gain should ramp linearly
    gains = []
    for step in range(5):
        gains.append(mgr.recovery_gain_scale)
        mgr.step_recovery()

    # Gains should be monotonically increasing
    for i in range(1, len(gains)):
        assert gains[i] >= gains[i-1], f"Gain not increasing at step {i}"

    assert mgr.current_mode == NavigationMode.GNSS_FIXED


def test_gnss_timeout_via_imu_tick():
    """Test that IMU tick timeout triggers GNSS outage detection."""
    cfg = NavigationModeConfig(gnss_timeout_s=2.0)
    mgr = NavigationModeManager(cfg)

    trusted = _make_report(0.9, GnssClassification.TRUSTED)

    # Normal GNSS update
    mgr.update_with_gnss(trusted, timestamp_ns=1_000_000_000)
    assert mgr.current_mode == NavigationMode.GNSS_FIXED

    # IMU ticks for 3 seconds without GNSS
    mgr.update_without_gnss(timestamp_ns=2_500_000_000)
    assert mgr.current_mode == NavigationMode.GNSS_FIXED  # 1.5s < 2.0s timeout

    mgr.update_without_gnss(timestamp_ns=4_000_000_000)  # 3.0s > 2.0s timeout
    assert mgr.current_mode == NavigationMode.TRANSITION_TO_DR


def test_transition_history():
    """Test that transition events are logged correctly."""
    cfg = NavigationModeConfig(outage_persistence_count=1)
    mgr = NavigationModeManager(cfg)

    rejected = _make_report(0.0, GnssClassification.REJECTED)

    mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)

    history = mgr.transition_history
    assert len(history) >= 2
    assert history[0].from_mode == NavigationMode.GNSS_FIXED
    assert history[0].to_mode == NavigationMode.TRANSITION_TO_DR


def test_reset():
    """Test state machine reset restores to initial mode."""
    cfg = NavigationModeConfig(outage_persistence_count=1)
    mgr = NavigationModeManager(cfg)

    rejected = _make_report(0.0, GnssClassification.REJECTED)
    mgr.update_with_gnss(rejected, timestamp_ns=1_000_000_000)
    mgr.update_with_gnss(rejected, timestamp_ns=2_000_000_000)
    assert mgr.current_mode == NavigationMode.DEAD_RECKONING

    mgr.reset()
    assert mgr.current_mode == NavigationMode.GNSS_FIXED
    assert len(mgr.transition_history) == 0
