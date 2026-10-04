import numpy as np
import os
from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network

synced = load_two_wheeler_session("data/raw/two_wheeler", "session2")
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

dt = 0.1
calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
if mag is not None:
    calib.calibrate_magnetometer(mag[:1200])

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
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)
if calib.mag_is_calibrated:
    fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
    fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
    fusion.calib.mag_is_calibrated = True
    fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

print(f"Initial heading: {gt_heading[0]:.1f}")
print("t(s) | H_est | H_gt | V_gt | AI_spd | is_stopped")

for i in range(1, 700):
    h_rad = np.radians(gt_heading[i])
    vel_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])

    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=mag[i] if mag is not None else None,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=vel_enu,
        is_gnss_available=True,
        timestamp=i * dt
    )

    if i % 50 == 0 or i == 694:
        h_est = fusion.ekf.get_euler_angles_deg()[2]
        print(f"{i*dt:5.1f} | {h_est:6.1f} | {gt_heading[i]:6.1f} | {speed[i]:5.2f} | {res['ai_speed']:5.2f}")
