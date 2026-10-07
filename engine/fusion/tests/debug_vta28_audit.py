import sys, os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import ProductionMobileFusionEngine
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

s_df, v_df = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta28")
dt = 0.1
synced = preprocess_session(s_df, v_df, target_dt=dt)

fusion = ProductionMobileFusionEngine(dt=dt, current_vehicle_type="car")

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)

N = len(synced)
speed_scales = []
ai_speeds = []
gnss_speeds = []
has_nan = False

for i in range(N):
    try:
        fusion.step(
            acc_raw=synced[["acc_x", "acc_y", "acc_z"]].iloc[i].values,
            gyro_raw=synced[["gyro_x", "gyro_y", "gyro_z"]].iloc[i].values,
            mag_raw=None,
            gnss_pos_enu=np.array([e_gt[i], n_gt[i], u_gt[i]]),
            gnss_vel_enu=None,
            gnss_heading_rad=None,
            is_gnss_available=True,
            ai_speed=synced["ai_speed"].iloc[i] if "ai_speed" in synced else None,
            is_stopped=bool(synced["is_stopped"].iloc[i]),
            lean_angle_rad=0.0
        )
    except Exception as e:
        print(f"Exception at {i}: {e}")
    if np.any(np.isnan(fusion.ekf.p)): has_nan = True
    speed_scales.append(getattr(fusion, 'speed_scale', 1.0))
    if "ai_speed" in synced: ai_speeds.append(synced["ai_speed"].iloc[i])
    gnss_speeds.append(np.linalg.norm(np.array([synced["speed"].iloc[i]]))) # Assuming gt speed or similar? Wait, GT speed is not straight from gps. I'll use the 'speed' or 'gt_speed'.

speed_scales = np.array(speed_scales)
print("Speed Scale initial:", speed_scales[0])
print("Speed Scale final:", speed_scales[-1])
print("Speed Scale mean:", np.mean(speed_scales))
print("NaNs detected:", has_nan)

