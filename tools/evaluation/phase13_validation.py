import numpy as np
import sys
import os

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
    use_adaptive_trust: bool = False,
    use_nhc: bool = False
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
    nhc_updates, nhc_rejections = 0, 0
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
            # Optionally apply Non-Holonomic Constraints (NHC) at 20 Hz
            if use_nhc and (i % 5 == 0):
                res_nhc = eskf.update_kinematic_constraints(lateral_variance=4.0, vertical_variance=4.0, gate=3.0)
                if res_nhc.accepted:
                    nhc_updates += 1
                else:
                    nhc_rejections += 1

            if use_ml and ml_estimator is not None:
                ml_estimator.add_vehicle_frame_sample(tuple(trajectory["accel_v"][i]), tuple(trajectory["gyro_v"][i]))
                if ml_estimator.should_update():
                    v_ml_raw, v_var_raw = ml_estimator.estimate_velocity()
                    v_ml = v_ml_raw - (-0.384) 
                    
                    if use_adaptive_trust:
                        v_var = calibrator.calibrate(v_var_raw)
                    else:
                        v_var = v_var_raw * 2.068
                        
                    R_v2n = quat_to_rotation_matrix(ins.state.attitude_q_v2n)
                    R_n2v = R_v2n.T
                    H = np.zeros((1, 15)); H[0, 3:6] = R_n2v[0, :]
                    innovation = v_ml - (R_n2v @ np.array(ins.state.velocity_mps))[0]

                    if use_adaptive_trust:
                        penalty = trust_monitor.evaluate_update(innovation)
                        v_var = v_var * penalty
                    
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
        "nhc_updates": nhc_updates
    }

def main():
    gen = SyntheticTrajectoryGenerator(dt=0.01, seed=42)
    
    print(f"{'Scenario':<15} | {'ESKF-Only':<10} | {'Phase 12':<10} | {'Phase 13 (NHC)':<15} | {'NHC Acc'}")
    print("-" * 80)
    
    scenarios = [
        ("15s @ 10m/s", 15.0, 10.0),
        ("15s @ 20m/s", 15.0, 20.0),
        ("15s @ 30m/s", 15.0, 30.0), # Transient failure case
        ("30s @ 30m/s", 30.0, 30.0) 
    ]
    
    for name, out, spd in scenarios:
        traj = gen.generate_straight_accel_decel(duration=50.0, max_speed=spd)
        
        r_eskf = run_ablation(traj, 10.0, 10.0+out, use_ml=False)
        r_p12 = run_ablation(traj, 10.0, 10.0+out, use_ml=True, use_adaptive_trust=True, use_nhc=False)
        r_p13 = run_ablation(traj, 10.0, 10.0+out, use_ml=True, use_adaptive_trust=True, use_nhc=True)
        
        print(f"{name:<15} | {r_eskf['drift']:6.1f}m    | {r_p12['drift']:6.1f}m    | {r_p13['drift']:6.1f}m {r_p13['rmse']:4.1f}m/s | {r_p13['nhc_updates']} / {int(out*20)}")
        
if __name__ == "__main__":
    main()
