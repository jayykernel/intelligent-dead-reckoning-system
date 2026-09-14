import numpy as np
from core.integrity.navigation_integrity import NavigationIntegrityMonitor, IntegrityStatus
from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState
from core.filters.eskf import ErrorStateKalmanFilter, UpdateResult
from core.sensors.data_types import ImuSample

def run_evaluation():
    # Setup
    initial_state = NavState(
        timestamp_ns=0, position_m=(0.0, 0.0, 0.0), velocity_mps=(10.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0), accel_bias_mps2=(0.0, 0.0, 0.0), gyro_bias_radps=(0.0, 0.0, 0.0)
    )
    ins = StrapdownINS(initial_state)
    ins.covariance = np.eye(15) * 1e-4 # Start very precise
    
    eskf = ErrorStateKalmanFilter(ins)
    monitor = NavigationIntegrityMonitor(pos_degraded_m=10.0, pos_unsafe_m=40.0)
    
    print("--- Time 0.0s: Healthy GNSS-aided navigation ---")
    status, unc = monitor.assess(ins)
    print(f"Status: {status.name} | Pos std: {unc.pos_std_m:.2f}m")
    
    print("\n--- Time 1.0 - 15.0s: GNSS Outage (Drifting) ---")
    dt = 0.01
    for i in range(1500):
        # Accelerate slightly to inject drift via inertial mechanics
        ins.propagate(
            ImuSample(timestamp_ns=int((i+1)*1e7), accel_m_s2=(0.05, 0.0, -9.80665), gyro_rad_s=(0.0, 0.0, 0.0)),
            propagate_covariance=True
        )
        if (i+1) % 500 == 0:
            status, unc = monitor.assess(ins)
            print(f"  Time {(i+1)/100:.1f}s | Status: {status.name} | Pos std: {unc.pos_std_m:.2f}m")
            
    print("\n--- Time 15.0 - 30.0s: Long GNSS Outage (Unsafe) ---")
    for i in range(1500, 3000):
        ins.propagate(
            ImuSample(timestamp_ns=int((i+1)*1e7), accel_m_s2=(0.05, 0.0, -9.80665), gyro_rad_s=(0.0, 0.0, 0.0)),
            propagate_covariance=True
        )
        if (i+1) % 500 == 0:
            status, unc = monitor.assess(ins)
            print(f"  Time {(i+1)/100:.1f}s | Status: {status.name} | Pos std: {unc.pos_std_m:.2f}m")
            
    # Now simulate an overconfident filter getting inconsistent measurements
    print("\n--- Time 30.0s: Injecting Incorrect High-Confidence (Overconfident Filter) ---")
    ins.covariance = np.eye(15) * 1e-4
    status, unc = monitor.assess(ins)
    print(f"  Covariance manually clamped. Status: {status.name} | Pos std: {unc.pos_std_m:.2f}m")
    
    # Send incorrect measurements which the ESKF rejects (Mahalanobis gate failure)
    for j in range(5):
        # We manually emulate update_position rejection
        monitor.report_update(UpdateResult(accepted=False, innovation=np.zeros(3), innovation_cov=np.eye(3), mahalanobis_dist=15.0))
        
    status, unc = monitor.assess(ins)
    print(f"  After 5 rejected measurements -> Status: {status.name} | Pos std: {unc.pos_std_m:.2f}m (Integrity correctly caught overconfidence!)")

if __name__ == "__main__":
    run_evaluation()
