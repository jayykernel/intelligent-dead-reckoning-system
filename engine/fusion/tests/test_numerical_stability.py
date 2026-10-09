import numpy as np
import pytest
import sys
import os

# Add engine directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.config.constants import GRAVITY_MS2
from engine.fusion.ekf import ErrorStateEKF
from engine.nhc_zupt.lean_ekf import LeanAngleEKF

def test_long_ins_propagation():
    """Test 10,000 steps of pure INS propagation without GNSS updates."""
    ekf = ErrorStateEKF(dt=0.1)

    # 10,000 steps of pure INS
    acc = np.array([0.1, -0.05, GRAVITY_MS2])
    gyro = np.array([0.01, -0.01, 0.02])

    for i in range(10000):
        # Stress with extreme values occasionally
        if i % 1000 == 0:
            ekf.predict(acc * 50.0, gyro * 50.0)
        else:
            ekf.predict(acc, gyro)

        # Verify state and covariance remain valid and positive definite
        if i % 500 == 0 or i == 9999:
            ekf._validate_state()
            w, _ = np.linalg.eigh(ekf.P)
            assert np.all(w >= 1e-9), f"Eigenvalues dropped below epsilon at step {i}: {w}"
            assert np.allclose(ekf.P, ekf.P.T, atol=1e-8), f"Covariance not symmetric at step {i}"

def test_invalid_input_rejection_ekf():
    """Verify fail-fast behavior on NaN and Inf inputs for ErrorStateEKF."""
    ekf = ErrorStateEKF()

    # Predict with NaN / Inf
    with pytest.raises(ValueError):
        ekf.predict(np.array([np.nan, 0.0, 9.8]), np.array([0.0, 0.0, 0.0]))
    with pytest.raises(ValueError):
        ekf.predict(np.array([0.0, 0.0, 9.8]), np.array([0.0, np.inf, 0.0]))

    # Update with NaN / Inf
    with pytest.raises(ValueError):
        ekf.update(np.array([np.nan, 0.0, 0.0]), np.zeros(3), np.eye(3), np.eye(3))
    with pytest.raises(ValueError):
        ekf.update(np.zeros(3), np.array([0.0, np.inf, 0.0]), np.eye(3), np.eye(3))
    with pytest.raises(ValueError):
        ekf.update(np.zeros(3), np.zeros(3), np.full((3, 3), np.nan), np.eye(3))
    with pytest.raises(ValueError):
        ekf.update(np.zeros(3), np.zeros(3), np.eye(3), np.full((3, 3), np.inf))

def test_repeated_prediction_update_cycles():
    """Test mixed prediction and multi-sensor update cycles under high dynamics."""
    ekf = ErrorStateEKF(dt=0.01)

    for cycle in range(500):
        # Predict with varying dynamics
        acc = np.array([
            2.0 * np.sin(cycle * 0.1),
            1.5 * np.cos(cycle * 0.1),
            GRAVITY_MS2 + 0.5 * np.sin(cycle * 0.05)
        ])
        gyro = np.array([
            0.1 * np.cos(cycle * 0.05),
            0.1 * np.sin(cycle * 0.05),
            0.3 * np.sin(cycle * 0.02)
        ])
        ekf.predict(acc, gyro)

        # Position update (simulated GNSS) every 10 steps
        if cycle % 10 == 0:
            z_pos = ekf.p + np.random.normal(0, 0.5, 3)
            h_pos = ekf.p
            H_pos = np.zeros((3, 15))
            H_pos[0:3, 0:3] = np.eye(3)
            R_pos = np.eye(3) * 2.0
            ekf.update(z_pos, h_pos, H_pos, R_pos)

        # Velocity update every 5 steps
        if cycle % 5 == 0:
            z_vel = ekf.v + np.random.normal(0, 0.1, 3)
            h_vel = ekf.v
            H_vel = np.zeros((3, 15))
            H_vel[0:3, 3:6] = np.eye(3)
            R_vel = np.eye(3) * 0.1
            ekf.update(z_vel, h_vel, H_vel, R_vel)

        # ZUPT update when "stopped"
        if 200 <= cycle <= 250:
            z_zupt = np.zeros(3)
            h_zupt = ekf.v
            H_zupt = np.zeros((3, 15))
            H_zupt[0:3, 3:6] = np.eye(3)
            R_zupt = np.eye(3) * 0.01
            ekf.update(z_zupt, h_zupt, H_zupt, R_zupt)

        ekf._validate_state()
        w, _ = np.linalg.eigh(ekf.P)
        assert np.all(w >= 1e-9)

def test_covariance_eigenvalue_flooring_recovery():
    """Verify that ill-conditioned or near-singular covariance is properly recovered."""
    ekf = ErrorStateEKF()

    # Artificially corrupt covariance with near-zero and negative eigenvalues
    bad_P = np.zeros((15, 15))
    bad_P[0, 0] = -1.0
    bad_P[1, 1] = 1e-15
    bad_P[2, 2] = 10.0
    ekf.P = bad_P

    # Symmetrize and floor
    ekf._ensure_positive_definite(epsilon=1e-9)
    ekf._validate_state()

    w, _ = np.linalg.eigh(ekf.P)
    assert np.all(w >= 1e-9), f"Flooring failed to enforce min eigenvalue: {w}"
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-8)

def test_lean_angle_ekf_numerical_stability():
    """Test LeanAngleEKF numerical stability, Joseph update, and input validation."""
    lean_ekf = LeanAngleEKF(dt=0.1)

    # 5,000 steps of prediction and updates
    for i in range(5000):
        gyro_y = 0.05 * np.sin(i * 0.02)
        lean_ekf.predict(gyro_y)

        if i % 2 == 0:
            acc_x = 1.0 * np.sin(i * 0.02)
            acc_z = GRAVITY_MS2 + 0.2 * np.cos(i * 0.01)
            speed = max(0.0, 10.0 + 5.0 * np.sin(i * 0.05))
            gyro_z = 0.1 * np.cos(i * 0.02)
            lean_ekf.update(acc_x, acc_z, speed=speed, gyro_z=gyro_z)

        lean_ekf._validate_state()
        w, _ = np.linalg.eigh(lean_ekf.P)
        assert np.all(w >= 1e-9)
        assert np.allclose(lean_ekf.P, lean_ekf.P.T, atol=1e-8)

def test_lean_angle_ekf_invalid_input_rejection():
    """Verify LeanAngleEKF raises on NaN/Inf inputs."""
    lean_ekf = LeanAngleEKF()

    with pytest.raises(ValueError):
        lean_ekf.predict(np.nan)
    with pytest.raises(ValueError):
        lean_ekf.predict(np.inf)

    with pytest.raises(ValueError):
        lean_ekf.update(np.nan, 9.8)
    with pytest.raises(ValueError):
        lean_ekf.update(0.0, np.inf)
    with pytest.raises(ValueError):
        lean_ekf.update(0.0, 9.8, speed=np.nan)
    with pytest.raises(ValueError):
        lean_ekf.update(0.0, 9.8, gyro_z=np.inf)
    with pytest.raises(ValueError):
        lean_ekf.update(0.0, 9.8, g=np.nan)

if __name__ == "__main__":
    pytest.main([__file__])
