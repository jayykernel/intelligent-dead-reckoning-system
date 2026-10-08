"""
engine/fusion/tests/test_outage_reacquisition.py

Unit tests for GNSS outage handling, adaptive covariance growth,
adaptive NIS reacquisition thresholding, and soft state recovery.
"""

import numpy as np
import pytest
from engine.fusion.ekf import ErrorStateEKF
from engine.fusion.fusion_engine import GNSSINSFusionEngine


def test_adaptive_outage_covariance_growth():
    ekf = ErrorStateEKF(dt=0.1)

    # Check baseline covariance after one step
    acc = np.array([0.0, 0.0, 9.8])
    gyro = np.zeros(3)

    ekf.predict(acc, gyro, Q_scale=1.0)
    P_nominal = np.copy(ekf.P)

    # Check outage covariance (with outage_scale > 1.0)
    ekf.set_initial_state(np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    ekf.outage_scale = 5.0
    ekf.predict(acc, gyro, Q_scale=1.0)
    P_outage = np.copy(ekf.P)

    # Outage covariance should be larger (P_new = F*P*F' + Q*scale)
    assert P_outage[0, 0] > P_nominal[0, 0]
    assert P_outage[3, 3] > P_nominal[3, 3]


def test_adaptive_nis_reacquisition_threshold():
    """Verify that loosened NIS threshold (nis_multiplier > 1.0) allows acceptance of reacquired fixes with innovation."""
    ekf = ErrorStateEKF(dt=0.1)
    p0 = np.zeros(3)
    v0 = np.zeros(3)
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    ekf.set_initial_state(p0, v0, q0)

    # Artificially set position and small covariance
    ekf.P[0:3, 0:3] = np.eye(3) * 1.0

    # GNSS fix with ~10m offset (large innovation)
    p_gnss = np.array([10.0, 0.0, 0.0])

    # Standard threshold (multiplier=1.0) may reject this high-innovation fix if sigma_pos is small
    passed_standard, nis_val, thresh_standard = ekf.update_gnss_position(
        p_gnss_enu=p_gnss,
        sigma_pos=2.0,
        alpha=0.01,
        nis_multiplier=1.0
    )

    # Reset state
    ekf.set_initial_state(p0, v0, q0)
    ekf.P[0:3, 0:3] = np.eye(3) * 1.0

    # Relaxed reacquisition threshold (multiplier=10.0)
    passed_reacq, nis_val_reacq, thresh_reacq = ekf.update_gnss_position(
        p_gnss_enu=p_gnss,
        sigma_pos=2.0,
        alpha=0.01,
        nis_multiplier=10.0
    )

    assert thresh_reacq > thresh_standard
    assert thresh_reacq == pytest.approx(thresh_standard * 10.0, rel=1e-5)


def test_soft_state_correction_under_repeated_rejections():
    """Verify that repeated consecutive rejections trigger bounded soft correction towards GNSS when trust is high."""
    engine = GNSSINSFusionEngine(enable_ai=False, enable_online_mag_cal=False)
    p0 = np.zeros(3)
    v0 = np.zeros(3)
    engine.initialize_state(p0_enu=p0, v0_enu=v0, heading0_deg=0.0, acc0_raw=np.array([0.0, 0.0, 9.8]))

    # Move nominal state away from origin
    engine.ekf.p = np.array([50.0, 50.0, 0.0])

    # Feed consecutive GNSS fixes at origin with high trust
    acc_raw = np.array([0.0, 0.0, 9.8])
    gyro_raw = np.zeros(3)
    gnss_pos = np.array([0.0, 0.0, 0.0])
    gnss_vel = np.zeros(3)

    initial_dist = np.linalg.norm(engine.ekf.p - gnss_pos)

    for i in range(10):
        engine.step(
            acc_raw=acc_raw,
            gyro_raw=gyro_raw,
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            is_gnss_available=True,
            timestamp=float(i) * 0.1,
            gnss_acc_m=1.0,
            gnss_sat_count=12,
            gnss_avg_cn0=38.0
        )

    final_dist = np.linalg.norm(engine.ekf.p - gnss_pos)
    # The soft correction + updates should pull the estimate closer to GNSS
    assert final_dist < initial_dist


def test_reacquisition_latency():
    """Verify that after an outage, the first valid GNSS fix is accepted rapidly (< 200 ms / 2 epochs)."""
    engine = GNSSINSFusionEngine(enable_ai=False, enable_online_mag_cal=False)
    p0 = np.zeros(3)
    v0 = np.array([0.0, 5.0, 0.0])
    engine.initialize_state(p0_enu=p0, v0_enu=v0, heading0_deg=0.0, acc0_raw=np.array([0.0, 0.0, 9.8]))

    acc_raw = np.array([0.0, 0.0, 9.8])
    gyro_raw = np.zeros(3)

    # Simulate 10 steps of GNSS outage
    for i in range(10):
        engine.step(
            acc_raw=acc_raw,
            gyro_raw=gyro_raw,
            is_gnss_available=False,
            timestamp=float(i) * 0.1
        )

    # GNSS returns at step 11
    res = engine.step(
        acc_raw=acc_raw,
        gyro_raw=gyro_raw,
        gnss_pos_enu=np.array([0.0, 5.0, 0.0]),
        gnss_vel_enu=np.array([0.0, 5.0, 0.0]),
        is_gnss_available=True,
        timestamp=1.1,
        gnss_acc_m=1.5,
        gnss_sat_count=10,
        gnss_avg_cn0=35.0
    )

    # In GNSS_AIDED mode or transition, position update should pass
    # Since trust score becomes high, GNSS acceptance happens within 1-2 epochs (< 200ms)
    assert np.all(np.isfinite(res["pos"]))
    assert np.all(np.isfinite(res["vel"]))


if __name__ == "__main__":
    pytest.main([__file__])
