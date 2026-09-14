import pytest
import numpy as np
from core.integrity.navigation_integrity import NavigationIntegrityMonitor, IntegrityStatus
from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState
from core.filters.eskf import UpdateResult

def create_ins():
    return StrapdownINS(
        NavState(
            timestamp_ns=0,
            position_m=(0.0, 0.0, 0.0),
            velocity_mps=(0.0, 0.0, 0.0),
            attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
            accel_bias_mps2=(0.0, 0.0, 0.0),
            gyro_bias_radps=(0.0, 0.0, 0.0)
        )
    )

def test_initial_healthy_state():
    monitor = NavigationIntegrityMonitor()
    ins = create_ins()
    ins.covariance = np.eye(15) * 0.001  # Tiny uncertainty
    
    status, unc = monitor.assess(ins)
    assert status == IntegrityStatus.HEALTHY
    assert unc.pos_std_m < 1.0

def test_degraded_position_uncertainty():
    monitor = NavigationIntegrityMonitor(pos_degraded_m=10.0)
    ins = create_ins()
    # position trace = 300 => std = sqrt(300) = ~17.3m
    ins.covariance[0:3, 0:3] = np.eye(3) * 100.0
    
    status, unc = monitor.assess(ins)
    assert status == IntegrityStatus.DEGRADED

def test_unsafe_velocity_uncertainty():
    monitor = NavigationIntegrityMonitor(vel_unsafe_mps=10.0)
    ins = create_ins()
    # velocity trace = 300 => std = sqrt(300) = ~17.3 m/s
    ins.covariance[3:6, 3:6] = np.eye(3) * 100.0
    
    status, unc = monitor.assess(ins)
    assert status == IntegrityStatus.UNSAFE

def test_overconfident_but_wrong_continuous_rejection():
    monitor = NavigationIntegrityMonitor(unsafe_reject_threshold=0.5)
    ins = create_ins()
    ins.covariance = np.eye(15) * 0.0001 # Extremely confident
    
    # 3 rejections out of 3 updates => 100% rejection rate
    monitor.report_update(UpdateResult(accepted=False, innovation=np.zeros(2), innovation_cov=np.eye(2), mahalanobis_dist=10.0))
    monitor.report_update(UpdateResult(accepted=False, innovation=np.zeros(2), innovation_cov=np.eye(2), mahalanobis_dist=15.0))
    monitor.report_update(UpdateResult(accepted=False, innovation=np.zeros(2), innovation_cov=np.eye(2), mahalanobis_dist=20.0))
    
    status, unc = monitor.assess(ins)
    assert status == IntegrityStatus.UNSAFE
    # Uncertainty itself is falsely low, the monitor flags it!
    assert unc.pos_std_m < 1.0 

def test_recovery_from_unsafe():
    monitor = NavigationIntegrityMonitor(unsafe_reject_threshold=0.5)
    ins = create_ins()
    ins.covariance = np.eye(15) * 0.0001 
    
    # Rejections
    for _ in range(5):
        monitor.report_update(UpdateResult(accepted=False, innovation=np.zeros(1), innovation_cov=np.eye(1), mahalanobis_dist=10.0))
    assert monitor.assess(ins)[0] == IntegrityStatus.UNSAFE
    
    # Valid GNSS recovers stability
    for _ in range(8):
        monitor.report_update(UpdateResult(accepted=True, innovation=np.zeros(1), innovation_cov=np.eye(1), mahalanobis_dist=1.0))
        
    assert monitor.assess(ins)[0] == IntegrityStatus.HEALTHY
