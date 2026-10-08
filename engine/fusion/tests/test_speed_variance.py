"""
Unit tests for AI forward speed measurement variance coupling with heading uncertainty.
"""

import numpy as np
import pytest
from engine.fusion.ekf import ErrorStateEKF


def test_ai_speed_variance_coupling_low_uncertainty():
    ekf = ErrorStateEKF(dt=0.1)
    p0 = np.zeros(3)
    v0 = np.array([0.0, 5.0, 0.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    ekf.set_initial_state(p0, v0, q0)

    # Initial P has default uncertainties
    # Set low heading uncertainty (yaw variance near 0)
    passed, nis, gate = ekf.update_ai_forward_speed(
        speed_fwd=5.0,
        sigma_speed=1.0,
        heading_uncertainty=0.01,
        timestamp=1.0
    )
    assert passed
    assert nis >= 0.0


def test_ai_speed_variance_coupling_high_uncertainty():
    ekf = ErrorStateEKF(dt=0.1)
    p0 = np.zeros(3)
    v0 = np.array([0.0, 5.0, 0.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    ekf.set_initial_state(p0, v0, q0)

    # High heading uncertainty (yaw variance = 1.0)
    passed, nis, gate = ekf.update_ai_forward_speed(
        speed_fwd=5.0,
        sigma_speed=1.0,
        heading_uncertainty=1.0,
        timestamp=1.0
    )
    assert passed
    assert nis >= 0.0


def test_ai_speed_variance_scaling_behavior():
    """Verify that higher heading uncertainty yields lower Kalman gain / smaller state correction."""
    # Run two identical filters, one with low uncertainty and one with high uncertainty
    ekf1 = ErrorStateEKF(dt=0.1)
    ekf2 = ErrorStateEKF(dt=0.1)

    p0 = np.zeros(3)
    v0 = np.array([0.0, 2.0, 0.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])

    ekf1.set_initial_state(p0, v0, q0)
    ekf2.set_initial_state(p0, v0, q0)

    # Apply innovation: measurement is 10.0 m/s
    ekf1.update_ai_forward_speed(speed_fwd=10.0, sigma_speed=1.0, heading_uncertainty=0.0)
    ekf2.update_ai_forward_speed(speed_fwd=10.0, sigma_speed=1.0, heading_uncertainty=1.0)

    # ekf1 (low uncertainty factor) should trust the speed measurement more than ekf2
    # Therefore, ekf1 velocity should be corrected more towards 10.0 m/s than ekf2
    assert ekf1.v[1] > ekf2.v[1]


if __name__ == "__main__":
    pytest.main([__file__])
