import numpy as np
from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine

synced = load_two_wheeler_session("data/raw/two_wheeler", "session2")
N = len(synced)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

dt = 0.1
calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
gt_heading = synced["gt_heading"].values
if np.any(np.isnan(gt_heading)):
    gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
heading_rad = np.radians(gt_heading[0])
v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="two_wheeler", k=1000.0)
fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

# Configure ZUPT thresholds for two-wheeler
fusion.constrained_ins.zupt_acc_threshold = 2.5
fusion.constrained_ins.zupt_gyro_threshold = 0.20

outage_start = min(int(N * 0.4), 3000)

for i in range(1, outage_start + 1):
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
    h_rad = np.radians(gt_heading[i])
    vel_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])
    
    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=vel_enu,
        is_gnss_available=True,
        timestamp=i * dt
    )

h_est = fusion.ekf.get_euler_angles_deg()[2]
print(f"At outage start t={outage_start*dt:.1f}s | Speed: {speed[outage_start]:.1f}m/s | H_est: {h_est:.1f} | H_gt: {gt_heading[outage_start]:.1f}")
