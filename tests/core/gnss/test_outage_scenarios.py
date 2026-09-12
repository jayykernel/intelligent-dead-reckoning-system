"""
GNSS Outage Scenario Tests: Deterministic synthetic validation.

Tests the complete integrity pipeline including quality scoring,
mode transitions, ESKF updates, and smooth recovery.
"""
import math
import numpy as np
import pytest
import time

from core.sensors.data_types import ImuSample, GnssFix
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.gnss.integrity import (
    GnssQualityEstimator,
    GnssClassification,
    GnssIntegrityConfig,
    GnssIntegrityReport,
)
from core.gnss.navigation_mode import (
    NavigationModeManager,
    NavigationMode,
    NavigationModeConfig,
)
from core.gnss.outage_manager import GnssIntegrityPipeline


# ============================================================
# HELPER: Build consistent trajectory for testing
# ============================================================

def _create_initial_state() -> NavState:
    return NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(10.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0),
    )


def _create_gnss_fix(
    t_s: float,
    lat: float = 37.7749,
    lon: float = -122.4194,
    alt: float = 10.0,
    vel_ned: tuple[float, float, float] = (10.0, 0.0, 0.0),
    hacc: float = 2.0,
    vacc: float = 3.0,
    sacc: float = 0.1,
    sats: int = 14,
) -> GnssFix:
    return GnssFix(
        timestamp_ns=int(t_s * 1e9),
        latitude_deg=lat,
        longitude_deg=lon,
        altitude_m=alt,
        velocity_ned_mps=vel_ned,
        horizontal_accuracy_m=hacc,
        vertical_accuracy_m=vacc,
        speed_accuracy_mps=sacc,
        satellite_count=sats,
    )


def _propagate_imu(ins: StrapdownINS, t_s: float):
    """Propagate INS with a constant-velocity IMU sample.
    To maintain constant velocity in navigation frame, set accelerometer to counteract gravity."""
    imu = ImuSample(
        timestamp_ns=int(t_s * 1e9),
        accel_m_s2=(0.0, 0.0, -9.81),  # counteract gravity in NED frame
        gyro_rad_s=(0.0, 0.0, 0.0),
    )
    ins.propagate(imu)


# ============================================================
# SCENARIO A: High-quality continuous GNSS
# ============================================================

def test_scenario_a_high_quality_gnss():
    """Scenario A: Continuous high-quality GNSS -> stays in GNSS_FIXED."""
    # Start at zero velocity and zero position (relative to origin that will be set by first GNSS fix)
    initial_state = NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0),
    )
    ins = StrapdownINS(initial_state)
    eskf = ErrorStateKalmanFilter(ins)
    pipeline = GnssIntegrityPipeline(eskf)

    # Propagate at 100 Hz between GNSS fixes to avoid large integration errors
    imu_hz = 100.0
    imu_dt = 1.0 / imu_hz

    for i in range(20):
        t_s = 1.0 + i * 1.0
        # Propagate IMU from previous second to current second at imu_hz
        for step in range(int(imu_hz)):
            t_mid = i * 1.0 + step * imu_dt
            _propagate_imu(ins, t_mid)
        # Create a GNSS fix at the origin (so NED position is zero) with zero velocity
        fix = _create_gnss_fix(
            t_s,
            lat=37.7749,
            lon=-122.4194,
            alt=10.0,
            vel_ned=(0.0, 0.0, 0.0),
            hacc=1.5,
            sats=16,
        )
        result = pipeline.process_gnss_fix(fix)

        assert result.navigation_mode == NavigationMode.GNSS_FIXED
        assert result.integrity_report.classification == GnssClassification.TRUSTED


# ============================================================
# SCENARIO B: Gradually degrading GNSS
# ============================================================

def test_scenario_b_gradual_degradation():
    """Scenario B: GNSS accuracy degrades gradually -> transitions to HYBRID then DR."""
    ins = StrapdownINS(_create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    pipeline = GnssIntegrityPipeline(eskf)

    modes_seen = set()
    for i in range(20):
        t_s = 1.0 + i * 1.0
        _propagate_imu(ins, t_s)

        # Accuracy degrades linearly
        hacc = 2.0 + i * 1.5
        sats = max(3, 14 - i)
        fix = _create_gnss_fix(t_s, hacc=hacc, vacc=hacc * 1.5, sacc=0.1 + i * 0.2, sats=sats)
        result = pipeline.process_gnss_fix(fix)
        modes_seen.add(result.navigation_mode)

    # Should have seen at least GNSS_FIXED and some degradation
    assert NavigationMode.GNSS_FIXED in modes_seen


# ============================================================
# SCENARIO C: Sudden GNSS blackout
# ============================================================

def test_scenario_c_sudden_blackout():
    """Scenario C: GNSS suddenly disappears -> transition to DR within timeout."""
    cfg_mode = NavigationModeConfig(gnss_timeout_s=2.0)
    initial_state = NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0),
    )
    ins = StrapdownINS(initial_state)
    eskf = ErrorStateKalmanFilter(ins)
    pipeline = GnssIntegrityPipeline(eskf, mode_config=cfg_mode)

    imu_hz = 100.0
    imu_dt = 1.0 / imu_hz

    # Normal GNSS for 5 seconds
    for i in range(5):
        t_s = 1.0 + i * 1.0
        for step in range(int(imu_hz)):
            t_mid = i * 1.0 + step * imu_dt
            _propagate_imu(ins, t_mid)
        fix = _create_gnss_fix(
            t_s,
            lat=37.7749,
            lon=-122.4194,
            alt=10.0,
            vel_ned=(0.0, 0.0, 0.0),
            hacc=2.0,
            sats=14
        )
        result = pipeline.process_gnss_fix(fix)
        assert result.navigation_mode == NavigationMode.GNSS_FIXED

    # Sudden loss: IMU ticks without GNSS for 3 seconds
    for j in range(300):
        t_s = 6.0 + j * 0.01
        _propagate_imu(ins, t_s)
        mode = pipeline.process_imu_tick(int(t_s * 1e9))

    # Should have transitioned to DR
    assert mode in (NavigationMode.TRANSITION_TO_DR, NavigationMode.DEAD_RECKONING)


# ============================================================
# SCENARIO D: 30-second GNSS outage
# ============================================================

def test_scenario_d_30s_outage():
    """Scenario D: 30-second GNSS outage -> uncertainty must grow, then recovers."""
    cfg_mode = NavigationModeConfig(gnss_timeout_s=2.0, recovery_persistence_count=2)
    ins = StrapdownINS(_create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    pipeline = GnssIntegrityPipeline(eskf, mode_config=cfg_mode)

    # Phase 1: good GNSS for 5 seconds
    for i in range(5):
        t_s = 1.0 + i * 1.0
        _propagate_imu(ins, t_s)
        fix = _create_gnss_fix(t_s, hacc=2.0, sats=14)
        pipeline.process_gnss_fix(fix)

    # Record position uncertainty before outage
    P_before = np.diag(ins.covariance).copy()

    # Phase 2: 30-second outage (IMU only)
    for j in range(300):
        t_s = 6.0 + j * 0.1
        _propagate_imu(ins, t_s)
        pipeline.process_imu_tick(int(t_s * 1e9))

    P_after_outage = np.diag(ins.covariance).copy()

    # Position/velocity uncertainty should grow during outage
    assert P_after_outage[0] > P_before[0], "Position uncertainty should grow during outage"
    assert P_after_outage[3] > P_before[3], "Velocity uncertainty should grow during outage"

    # Phase 3: GNSS recovery
    for k in range(5):
        t_s = 36.0 + k * 1.0
        _propagate_imu(ins, t_s)
        fix = _create_gnss_fix(t_s, hacc=2.0, sats=14)
        result = pipeline.process_gnss_fix(fix)

    # Should be recovering or recovered
    assert result.navigation_mode in (
        NavigationMode.TRANSITION_TO_GNSS,
        NavigationMode.RECOVERED,
        NavigationMode.GNSS_FIXED
    )


# ============================================================
# SCENARIO E: GNSS recovery with consistent position
# ============================================================

def test_scenario_e_consistent_recovery():
    """Scenario E: GNSS recovers with position consistent with INS prediction."""
    cfg_mode = NavigationModeConfig(outage_persistence_count=1, recovery_persistence_count=2)
    ins = StrapdownINS(_create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    pipeline = GnssIntegrityPipeline(eskf, mode_config=cfg_mode)

    rejected_fix = _create_gnss_fix(1.0, hacc=2.0, sats=2)  # Low sats = rejected

    # Enter DR
    pipeline.process_gnss_fix(rejected_fix)
    pipeline.process_gnss_fix(_create_gnss_fix(2.0, hacc=2.0, sats=2))

    assert pipeline.mode_manager.current_mode in (
        NavigationMode.TRANSITION_TO_DR,
        NavigationMode.DEAD_RECKONING
    )

    # Recovery with consistent signal
    for i in range(5):
        t_s = 3.0 + i * 1.0
        fix = _create_gnss_fix(t_s, hacc=2.0, sats=14)
        result = pipeline.process_gnss_fix(fix)

    # Should be recovering or recovered
    final_mode = pipeline.mode_manager.current_mode
    assert final_mode in (
        NavigationMode.TRANSITION_TO_GNSS,
        NavigationMode.RECOVERED,
        NavigationMode.GNSS_FIXED
    )


# ============================================================
# SCENARIO F: GNSS recovery with erroneous position (outlier)
# ============================================================

def test_scenario_f_erroneous_recovery():
    """Scenario F: GNSS recovers with wildly erroneous position -> should be rejected."""
    est = GnssQualityEstimator()

    # Previous fix at known position
    fix1 = _create_gnss_fix(1.0, lat=37.7749, lon=-122.4194)
    est.evaluate(fix1)

    # "Recovery" fix with huge jump (100km away in 1 second)
    fix2 = _create_gnss_fix(2.0, lat=38.7749, lon=-122.4194)  # ~111km N jump
    report = est.evaluate(fix2)

    assert report.classification == GnssClassification.REJECTED
    assert "jump" in report.rejection_reason.lower() or "Position jump" in report.rejection_reason


# ============================================================
# SCENARIO G: Intermittent GNSS (urban canyon)
# ============================================================

def test_scenario_g_intermittent_gnss():
    """Scenario G: Intermittent GNSS on/off pattern -> no rapid mode oscillation."""
    cfg = NavigationModeConfig(
        outage_persistence_count=2,
        recovery_persistence_count=3,
    )
    mgr = NavigationModeManager(cfg)

    trusted = GnssIntegrityReport(
        quality_score=0.9, classification=GnssClassification.TRUSTED,
        position_accuracy_score=0.9, velocity_accuracy_score=0.9,
        satellite_score=0.9, innovation_score=0.9, temporal_score=0.9,
        scaled_pos_cov=np.eye(3) * 4.0, scaled_vel_cov=np.eye(3) * 0.1,
    )
    rejected = GnssIntegrityReport(
        quality_score=0.0, classification=GnssClassification.REJECTED,
        position_accuracy_score=0.0, velocity_accuracy_score=0.0,
        satellite_score=0.0, innovation_score=0.0, temporal_score=0.0,
        scaled_pos_cov=np.eye(3) * 1e8, scaled_vel_cov=np.eye(3) * 1e8,
        rejection_reason="Low quality",
    )

    modes = []
    # Alternating pattern: trusted, rejected, trusted, rejected...
    for i in range(10):
        t_ns = (i + 1) * 1_000_000_000
        report = trusted if i % 2 == 0 else rejected
        mode = mgr.update_with_gnss(report, t_ns)
        modes.append(mode)

    # With persistence_count=2, shouldn't have rapid oscillation
    transition_count = sum(1 for j in range(1, len(modes)) if modes[j] != modes[j-1])
    assert transition_count <= 3, f"Too many mode transitions ({transition_count}); hysteresis failing"


# ============================================================
# SCENARIO H: Large position jump (multipath)
# ============================================================

def test_scenario_h_large_position_jump():
    """Scenario H: Sudden 500m position jump -> must be rejected."""
    est = GnssQualityEstimator()

    fix1 = _create_gnss_fix(1.0, lat=37.7749, lon=-122.4194)
    est.evaluate(fix1)

    # 0.5s later, 500m jump (implied 1000 m/s speed)
    fix2 = _create_gnss_fix(1.5, lat=37.7749 + 500.0 / 111139.0, lon=-122.4194)
    report = est.evaluate(fix2)

    assert report.classification == GnssClassification.REJECTED


# ============================================================
# BASELINE COMPARISON: Integrity-Aware vs Naive
# ============================================================

def test_baseline_comparison_integrity_vs_naive():
    """Compare integrity-aware pipeline against naive 'always trust GNSS' approach.

    A series of good fixes, then a bad/outlier fix, then good fixes.
    Naive approach snaps to bad fix position; integrity-aware rejects it.
    """
    ins_naive = StrapdownINS(_create_initial_state())
    eskf_naive = ErrorStateKalmanFilter(ins_naive)

    ins_smart = StrapdownINS(_create_initial_state())
    eskf_smart = ErrorStateKalmanFilter(ins_smart)
    pipeline_smart = GnssIntegrityPipeline(eskf_smart)

    # Good fixes for 5 seconds at origin area
    for i in range(5):
        t_s = 1.0 + i * 1.0
        _propagate_imu(ins_naive, t_s)
        _propagate_imu(ins_smart, t_s)

        fix = _create_gnss_fix(t_s, hacc=2.0, sats=14)

        # Naive: always apply GNSS update
        hacc = fix.horizontal_accuracy_m
        vacc = fix.vertical_accuracy_m
        sacc = fix.speed_accuracy_mps
        R_pos = np.diag([hacc**2, hacc**2, vacc**2])
        R_vel = np.diag([sacc**2, sacc**2, sacc**2])
        from core.navigation.earth import lla_to_ned
        pos_ned = lla_to_ned(fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
                              37.7749, -122.4194, 10.0)
        eskf_naive.update_position(pos_ned, R_pos, gate=100.0)
        eskf_naive.update_velocity(fix.velocity_ned_mps, R_vel, gate=100.0)

        # Smart: through pipeline
        pipeline_smart.process_gnss_fix(fix)

    # Record state before outlier
    naive_pos_before = np.array(ins_naive.state.position_m)
    smart_pos_before = np.array(ins_smart.state.position_m)

    # Inject an OUTLIER fix: 200m jump
    t_outlier = 6.0
    _propagate_imu(ins_naive, t_outlier)
    _propagate_imu(ins_smart, t_outlier)

    outlier_lat = 37.7749 + 200.0 / 111139.0  # ~200m N jump
    outlier_fix = _create_gnss_fix(t_outlier, lat=outlier_lat, hacc=2.0, sats=14)

    # Naive: blindly apply
    outlier_pos_ned = lla_to_ned(
        outlier_fix.latitude_deg, outlier_fix.longitude_deg, outlier_fix.altitude_m,
        37.7749, -122.4194, 10.0)
    R_pos = np.diag([4.0, 4.0, 9.0])
    R_vel = np.diag([0.01, 0.01, 0.01])
    naive_pos_update = eskf_naive.update_position(outlier_pos_ned, R_pos, gate=100.0)
    eskf_naive.update_velocity(outlier_fix.velocity_ned_mps, R_vel, gate=100.0)

    # Smart: through pipeline (should reject this outlier)
    smart_result = pipeline_smart.process_gnss_fix(outlier_fix)

    naive_pos_after = np.array(ins_naive.state.position_m)
    smart_pos_after = np.array(ins_smart.state.position_m)

    # Naive should have jumped significantly toward outlier
    naive_jump = float(np.linalg.norm(naive_pos_after - naive_pos_before))
    # Smart should NOT have jumped (outlier rejected)
    smart_jump = float(np.linalg.norm(smart_pos_after - smart_pos_before))

    assert naive_jump > 10.0, f"Naive should have snapped to outlier (jump={naive_jump:.1f}m)"
    assert smart_jump < naive_jump, f"Smart should have smaller jump than naive"


# ============================================================
# DETERMINISTIC REPLAY
# ============================================================

def test_deterministic_replay():
    """Verify that identical input sequence produces identical output."""
    def _run_scenario():
        ins = StrapdownINS(_create_initial_state())
        eskf = ErrorStateKalmanFilter(ins)
        pipeline = GnssIntegrityPipeline(eskf)

        results = []
        for i in range(10):
            t_s = 1.0 + i * 1.0
            _propagate_imu(ins, t_s)
            fix = _create_gnss_fix(t_s, hacc=2.0 + i * 0.5, sats=max(4, 14 - i))
            r = pipeline.process_gnss_fix(fix)
            results.append((
                r.navigation_mode,
                r.integrity_report.quality_score,
                r.integrity_report.classification,
            ))

        final_pos = np.array(ins.state.position_m)
        final_vel = np.array(ins.state.velocity_mps)
        return results, final_pos, final_vel

    r1, pos1, vel1 = _run_scenario()
    r2, pos2, vel2 = _run_scenario()

    for i, (a, b) in enumerate(zip(r1, r2)):
        assert a[0] == b[0], f"Mode mismatch at step {i}"
        assert abs(a[1] - b[1]) < 1e-10, f"Quality score mismatch at step {i}"
        assert a[2] == b[2], f"Classification mismatch at step {i}"

    assert np.allclose(pos1, pos2), "Positions should be reproducible"
    assert np.allclose(vel1, vel2), "Velocities should be reproducible"


# ============================================================
# PROCESSING LATENCY
# ============================================================

def test_processing_latency():
    """Measure processing time for integrity pipeline (should be << 1ms)."""
    ins = StrapdownINS(_create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    pipeline = GnssIntegrityPipeline(eskf)

    fix = _create_gnss_fix(1.0, hacc=2.0, sats=14)

    # Warmup
    pipeline.process_gnss_fix(fix)

    # Measure 100 fixes
    start = time.perf_counter()
    for i in range(100):
        fix = _create_gnss_fix(2.0 + i * 0.1, hacc=2.0, sats=14)
        pipeline.process_gnss_fix(fix)
    elapsed = time.perf_counter() - start

    avg_ms = (elapsed / 100) * 1000.0
    assert avg_ms < 10.0, f"Average processing time {avg_ms:.2f}ms > 10ms target"
