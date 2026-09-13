import math
import numpy as np
import pytest

from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.sensors.data_types import ImuSample

def test_forward_velocity_aiding():
    """Verify that a forward velocity measurement constrains velocity in vehicle frame."""
    
    ins = StrapdownINS()
    # Nominal velocity is zero. Vehicle is pitched up by 30 degrees.
    # We will simulate a forward velocity measurement of 10 m/s.
    
    from core.alignment.quaternion_utils import quat_from_euler
    # Pitch up 30 degrees: rotation around Vehicle Y axis
    ins.state.attitude_q_v2n = quat_from_euler(0.0, math.radians(30), 0.0)
    # The vehicle frame is pitched up. Forward (Vehicle X) points 30 deg up relative to NED North.
    # Therefore, 10m/s forward means: North = 10 * cos(30), Up(-Down) = 10 * sin(30).
    
    # Covariance set to 100 for velocity
    ins.covariance = np.eye(15)
    ins.covariance[3:6, 3:6] = np.eye(3) * 100.0
    
    eskf = ErrorStateKalmanFilter(ins)
    
    # Measure 10 m/s forward speed
    res = eskf.update_forward_velocity(10.0, variance=0.01, gate=float('inf'))
    
    assert res.accepted
    
    # Check updated velocity in NED
    vn = ins.state.velocity_mps
    
    exp_north = 10.0 * math.cos(math.radians(30))
    exp_down = -10.0 * math.sin(math.radians(30))
    
    # Should be close to constraints
    assert math.isclose(vn[0], exp_north, abs_tol=1e-1)
    assert math.isclose(vn[1], 0.0, abs_tol=1e-1)
    assert math.isclose(vn[2], exp_down, abs_tol=1e-1)

def test_forward_velocity_jacobian():
    """Verify analytical jacobian for forward velocity matches finite differences."""
    ins = StrapdownINS()
    ins.state.velocity_mps = (10.0, 5.0, -2.0)
    
    from core.alignment.quaternion_utils import quat_from_euler
    ins.state.attitude_q_v2n = quat_from_euler(0.1, -0.2, 0.5)
    
    eskf = ErrorStateKalmanFilter(ins)
    
    # We call the method just to get the analytical H
    # We'll mock _apply_measurement to capture H
    captured_H = None
    original_apply = eskf._apply_measurement
    def mock_apply(z, H, R, mahalanobis_gate):
        nonlocal captured_H
        captured_H = H
        from core.filters.eskf import UpdateResult
        import numpy as np
        return UpdateResult(True, np.zeros(1), np.zeros((1,1)), 0.0)

    eskf._apply_measurement = mock_apply
    eskf.update_forward_velocity(10.0, 1.0, couple_attitude=True)

    H_analytical = captured_H[0]

    # Compute finite differences
    delta = 1e-5
    H_fd = np.zeros(15)

    from core.alignment.quaternion_utils import quat_to_rotation_matrix, quat_multiply, quat_from_axis_angle

    def get_vx(state):
        R_v2n = quat_to_rotation_matrix(state.attitude_q_v2n)
        R_n2v = R_v2n.T
        v_v = R_n2v @ np.array(state.velocity_mps)
        return v_v[0]

    v_x_nom = get_vx(ins.state)

    import copy

    # Velocity perturbation
    for i in range(3):
        s_cpy = copy.deepcopy(ins.state)
        v = list(s_cpy.velocity_mps)
        v[i] += delta
        s_cpy.velocity_mps = tuple(v)
        v_x_pert = get_vx(s_cpy)
        H_fd[3+i] = (v_x_pert - v_x_nom) / delta

    # Attitude perturbation
    for i in range(3):
        s_cpy = copy.deepcopy(ins.state)
        d_theta = [0, 0, 0]
        d_theta[i] = delta
        angle = delta
        axis = np.array(d_theta) / angle
        q_err = quat_from_axis_angle(tuple(axis), angle)

        # q_true = q_nom * q_err
        s_cpy.attitude_q_v2n = quat_multiply(s_cpy.attitude_q_v2n, q_err)
        v_x_pert = get_vx(s_cpy)
        H_fd[6+i] = (v_x_pert - v_x_nom) / delta

    assert np.allclose(H_analytical[3:6], H_fd[3:6], rtol=1e-3, atol=1e-5)
    assert np.allclose(H_analytical[6:9], H_fd[6:9], rtol=1e-3, atol=1e-5)

