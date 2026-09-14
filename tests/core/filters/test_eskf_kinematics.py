import pytest
import numpy as np
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter

def create_initial_state():
    return NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        # Moving at 10m/s mostly North, but somehow acquired lateral 2m/s
        velocity_mps=(10.0, 2.0, -1.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0), # Identity: Vehicle matches NED 
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )

def test_nhc_constrains_lateral_velocity():
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    # Inflate covariance to see an immediate update effect
    eskf.ins.covariance[3:6, 3:6] = np.eye(3) * 1.0

    # Apply constraint (using couple_attitude=False for literal velocity squash)
    res = eskf.update_kinematic_constraints(lateral_variance=0.01, vertical_variance=0.01, gate=float('inf'), couple_attitude=False)
    
    assert res.accepted is True
    
    # Given q is identity, v_n = v_v. So v_n[1] was 2.0, now should be squashed towards 0.
    assert abs(eskf.ins.state.velocity_mps[1]) < 0.5
    assert abs(eskf.ins.state.velocity_mps[2]) < 0.5
    # Forward velocity should be unaffected because H projects lateral/vertical only
    assert abs(eskf.ins.state.velocity_mps[0] - 10.0) < 1e-4

def test_nhc_gating():
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    eskf.ins.covariance[3:6, 3:6] = np.eye(3) * 0.001
    
    # Gate is small, measurement error (innovation) is large (2.0 and -1.0), should reject
    res = eskf.update_kinematic_constraints(lateral_variance=0.01, vertical_variance=0.01, gate=1.0)
    assert res.accepted is False
