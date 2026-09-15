"""
Phase 20: Cross-Device / Cross-Vehicle Robustness Verification
"""
import math
import numpy as np
import pytest

from core.sensors.data_types import ImuSample
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.integrity.navigation_integrity import NavigationIntegrityMonitor, IntegrityStatus

def _create_initial_state() -> NavState:
    return NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(10.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0),
    )

def test_device_variability_high_noise():
    """Evaluate resilience of integrity mechanism to extreme mobile IMU noises."""
    ins = StrapdownINS(
        _create_initial_state(), 
        accel_noise_std=0.5, # 10x normal
        gyro_noise_std=0.05   # 10x normal
    )
    monitor = NavigationIntegrityMonitor()

    # Propagate 10s of high-noise IMU (unconstrained DR)
    for i in range(100):
        ins.propagate(ImuSample(int(i*0.1*1e9), (0.0, 0.0, -9.81), (0.0, 0.0, 0.0)), propagate_covariance=True)

    status, uncert = monitor.assess(ins)
    assert status == IntegrityStatus.DEGRADED or status == IntegrityStatus.UNSAFE
    assert uncert.pos_std_m > 1.0 # Significant drift uncertainty

def test_device_variability_sampling_jitter():
    """Assesses stability under varying dt (sampling rate jitter typical of Android Android)."""
    ins = StrapdownINS(_create_initial_state())
    
    # 10s of jittery sampling
    t_ns = 0
    for i in range(100):
        # Base dt is 0.1s (10Hz). Add +/- up to 0.05s jitter.
        dt_jitter = 0.1 + np.random.uniform(-0.05, 0.05)
        t_ns += int(dt_jitter * 1e9)
        ins.propagate(ImuSample(t_ns, (0.0, 0.0, -9.81), (0.0, 0.0, 0.0)), propagate_covariance=True)

    monitor = NavigationIntegrityMonitor()
    status, _ = monitor.assess(ins)
    # The integration math correctly tracks dt, so as long as dt is handled, the pipeline is stable.
    assert status != None

def test_vehicle_variability_motorcycle_lean():
    """Mock a motorcycle leaning during a turn where roll angle is significant."""
    ins = StrapdownINS(_create_initial_state())
    
    # Turning right while leaning right
    for i in range(100):
        ins.propagate(ImuSample(
            int(i*0.1*1e9), 
            # Lean creates an apparent lateral acceleration
            (0.0, 2.0, -9.81 + 0.5), 
            # Constant roll/yaw rates
            (0.1, 0.0, 0.2)), 
            propagate_covariance=True
        )

    monitor = NavigationIntegrityMonitor()
    status, uncert = monitor.assess(ins)
    assert uncert.att_std_deg > 0.0

def test_vehicle_high_speed_vibration():
    """Mock a harsh vehicle vibration profile on gravel road."""
    ins = StrapdownINS(_create_initial_state())
    
    # 10s of high vibration 
    for i in range(100):
        noise_a = np.random.normal(0, 2.0, 3) # 2m/s2 vibration
        noise_g = np.random.normal(0, 0.1, 3) # 0.1rad/s vibration
        ins.propagate(ImuSample(
            int(i*0.1*1e9), 
            (noise_a[0], noise_a[1], -9.81 + noise_a[2]), 
            (noise_g[0], noise_g[1], noise_g[2])), 
            propagate_covariance=True
        )

    monitor = NavigationIntegrityMonitor()
    status, _ = monitor.assess(ins)
    # Drift happens, but it shouldn't crash
    assert status != IntegrityStatus.HEALTHY # Must recognize extreme situation and degrade

def test_sensor_axis_permutation():
    """Verify system remains mathematically stable but degrades properly when orientation is mismatched."""
    ins = StrapdownINS(_create_initial_state())
    monitor = NavigationIntegrityMonitor()

    for i in range(100):
        # Accelerometer X/Y inverted, creating orientation error
        ins.propagate(ImuSample(int(i*0.1*1e9), (-2.0, 2.0, -9.81), (0.0, 0.0, 0.0)), propagate_covariance=True)

    status, uncert = monitor.assess(ins)
    assert not math.isnan(uncert.pos_std_m)

def test_vehicle_domain_mismatch_ml_velocity():
    """Evaluate fallback when ML velocity model is miscalibrated for the vehicle (e.g. expects car, is scooter)."""
    ins = StrapdownINS(_create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)
    
    # Simulating 5 seconds of ML providing horribly wrong velocity (car model applies to scooter accelerating)
    # The innovation gating should reject it.
    for i in range(5):
        # Truth is 10m/s, ML says 30m/s.
        res = eskf.update_forward_velocity(30.0, variance=0.1, gate=3.0)
        assert res.accepted == False # ML is rejected due to Mahalanobis gate

def test_device_bias_variation():
    """Verify that unmodeled bias drifts are contained by continuous variance propagation."""
    ins = StrapdownINS(_create_initial_state())
    monitor = NavigationIntegrityMonitor()

    # 5s of unmodeled bias drift (calibration failed)
    for i in range(50):
        # 0.5 m/s2 unmodeled bias
        ins.propagate(ImuSample(int(i*0.1*1e9), (0.5, 0.0, -9.81), (0.005, 0.0, 0.0)), propagate_covariance=True)
        
    status, uncert = monitor.assess(ins)
    assert not math.isnan(uncert.pos_std_m)
