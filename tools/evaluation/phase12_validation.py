import numpy as np
import sys
import os
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from core.models.dataset_generator import SyntheticTrajectoryGenerator
from core.models.velocity_estimator import VelocityEstimatorAPI
from core.filters.eskf import ErrorStateKalmanFilter
from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState
from core.sensors.data_types import ImuSample
from core.gnss.adaptive_trust import AdaptiveTrustMonitor, TrustState, UncertaintyCalibrator
from core.alignment.quaternion_utils import quat_to_rotation_matrix

def run_ablation(
    trajectory: dict, outage_start: float, outage_end: float,
    use_ml: bool = False,
    recalibrate_unc: bool = False,
    use_adaptive_trust: bool = False
):
    time = trajectory["time"]
    ins = StrapdownINS(NavState(0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    eskf = ErrorStateKalmanFilter(ins)
    
    trust_monitor = AdaptiveTrustMonitor()
    calibrator = UncertaintyCalibrator()

    model_path = os.path.join(os.path.dirname(__file__), "../../core/models/velocity_model.pth")
    if use_ml:
        ml_estimator = VelocityEstimatorAPI(model_path=model_path, window_size=100, update_interval=20)
        ml_estimator.samples_since_last_update = 20
    else:
        ml_estimator = None

    updates, rejections, fallbacks = 0, 0, 0
    vel_est_north = np.zeros(len(time))

    for i in range(len(time)):
        ts_ns = int(time[i] * 1e9)
        imu = ImuSample(ts_ns, tuple(trajectory["accel_v"][i]), tuple(trajectory["gyro_v"][i]))
        ins.propagate(imu, (1.0, 0.0, 0.0, 0.0), propagate_covariance=True)

        if time[i] < outage_start or time[i] > outage_end:
            eskf.update_velocity((trajectory["vel_v"][i][0], 0, 0), np.eye(3)*0.1)
            eskf.update_position((0, 0, 0), np.eye(3)*0.1)
            trust_monitor.reset()
        else:
            if use_ml and ml_estimator is not None:
                ml_estimator.add_vehicle_frame_sample(tuple(trajectory["accel_v"][i]), tuple(trajectory["gyro_v"][i]))
                if ml_estimator.should_update():
                    v_ml_raw, v_var_raw = ml_estimator.estimate_velocity()
                    v_ml = v_ml_raw - (-0.384) # Phase 10 bias correct
                    
                    if recalibrate_unc:
                        v_var = calibrator.calibrate(v_var_raw)
                    else:
                        v_var = v_var_raw * 2.068
                        
                    # Pre-calculate innovation natively
                    R_v2n = quat_to_rotation_matrix(ins.state.attitude_q_v2n)
                    R_n2v = R_v2n.T
                    H = np.zeros((1, 15)); H[0, 3:6] = R_n2v[0, :]
                    innovation = v_ml - (R_n2v @ np.array(ins.state.velocity_mps))[0]

                    if use_adaptive_trust:
                        penalty = trust_monitor.evaluate_update(innovation)
                        v_var = v_var * penalty
                    
                    # Estimate Mahalanobis
                    S = H @ eskf.ins.covariance @ H.T + np.array([[v_var]])
                    mahal = np.sqrt(innovation**2 / S[0,0])
                    
                    if mahal <= 3.0 and v_var != float('inf'):
                        res = eskf.update_forward_velocity(v_ml, v_var, gate=3.0, couple_attitude=False)
                        if res.accepted:
                            updates += 1
                        else:
                            rejections += 1
                    else:
                        rejections += 1
                        
                    if trust_monitor.state == TrustState.FALLBACK:
                        fallbacks += 1

        vel_est_north[i] = ins.state.velocity_mps[0]

    pos_gt = np.cumsum(trajectory["vel_v"][:, 0] * 0.01)
    pos_est = np.cumsum(vel_est_north * 0.01)
    return {
        "drift": abs(pos_est[-1] - pos_gt[-1]),
        "rmse": np.sqrt(np.mean((vel_est_north - trajectory["vel_v"][:, 0])**2)),
        "updates": updates,
        "rejections": rejections,
        "fallbacks": fallbacks
    }

def main():
    gen = SyntheticTrajectoryGenerator(dt=0.01, seed=42)
    scenarios = [
        ( "5s / 10 m/s",   5.0, 10.0),
        ( "5s / 20 m/s",   5.0, 20.0),
        ( "5s / 30 m/s",   5.0, 30.0),
        ("15s / 10 m/s",  15.0, 10.0),
        ("15s / 20 m/s",  15.0, 20.0),
        ("15s / 30 m/s",  15.0, 30.0),
        ("30s / 10 m/s",  30.0, 10.0),
        ("30s / 20 m/s",  30.0, 20.0),
        ("30s / 30 m/s",  30.0, 30.0),
    ]

    print(f"{'Scenario':<15} | {'ESKF-Only':<15} | {'Phase 11 ML':<15} | {'Phase 12 (Full)':<20} | {'Status'}")
    print("-" * 80)
    
    for name, out, spd in scenarios:
        traj = gen.generate_straight_accel_decel(duration=50.0, max_speed=spd)
        
        # 1. ESKF Only
        r1 = run_ablation(traj, 10.0, 10.0+out, use_ml=False)
        # 2. Phase 11
        r2 = run_ablation(traj, 10.0, 10.0+out, use_ml=True, recalibrate_unc=False, use_adaptive_trust=False)
        # 3. Phase 12 Full
        r3 = run_ablation(traj, 10.0, 10.0+out, use_ml=True, recalibrate_unc=True, use_adaptive_trust=True)
        
        # Classification
        status = "BENEFIT_PRESERVED" if r3['drift'] < r1['drift'] else "SAFE_FALLBACK"
        if r3['drift'] > r1['drift'] + 10.0:
            status = "FAILED"
            
        acc_pct = r3['updates'] / max(1, r3['updates'] + r3['rejections']) * 100
            
        print(f"{name:<15} | {r1['drift']:7.1f}m {r1['rmse']:4.1f}m/s | {r2['drift']:7.1f}m {r2['rmse']:4.1f}m/s | {r3['drift']:7.1f}m {r3['rmse']:4.1f}m/s ({int(acc_pct)}%) | {status}")
        
    print("\n---\nABLATION STUDY AT 30s / 30m/s (Catastrophic Case)")
    t = gen.generate_straight_accel_decel(duration=50.0, max_speed=30.0)
    
    a1 = run_ablation(t, 10.0, 40.0, use_ml=False)
    a2 = run_ablation(t, 10.0, 40.0, use_ml=True, recalibrate_unc=False, use_adaptive_trust=False)
    a3 = run_ablation(t, 10.0, 40.0, use_ml=True, recalibrate_unc=True, use_adaptive_trust=False)
    a4 = run_ablation(t, 10.0, 40.0, use_ml=True, recalibrate_unc=False, use_adaptive_trust=True)
    a5 = run_ablation(t, 10.0, 40.0, use_ml=True, recalibrate_unc=True, use_adaptive_trust=True)
    
    print(f"1. ESKF-only:                     {a1['drift']:7.1f}m")
    print(f"2. Baseline ESKF + ML (Phase 11): {a2['drift']:7.1f}m (Acc: {a2['updates']})")
    print(f"3. + Uncertainty Calibration only:{a3['drift']:7.1f}m (Acc: {a3['updates']})")
    print(f"4. + Innovation State Machine only: {a4['drift']:7.1f}m (Acc: {a4['updates']})")
    print(f"5. Combined Gate (Full Phase 12): {a5['drift']:7.1f}m (Acc: {a5['updates']})")
    
if __name__ == "__main__":
    main()
