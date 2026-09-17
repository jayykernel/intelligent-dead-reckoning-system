import numpy as np
import os, sys
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = synced["gt_speed"].values
speed = np.nan_to_num(speed, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

fusion = GNSSINSFusionEngine(dt=0.1, enable_ai=False)

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

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

print("Starting loop")
for i in range(1, 25):
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
    v0_enu = np.array([speed[i] * np.sin(np.radians(gt_heading[i])), speed[i] * np.cos(np.radians(gt_heading[i])), 0.0])
    res = fusion.step(acc_raw=acc[i], gyro_raw=gyro[i], gnss_pos_enu=pos_enu, gnss_vel_enu=v0_enu, is_gnss_available=True, timestamp=i*0.1)
    print(f"Step {i:02d}: Z_est={fusion.ekf.p[2]:.2f}, Z_gt={pos_enu[2]:.2f}, Pos Passed={res['gnss_pos_passed']}, Vel Passed={res['gnss_vel_passed']}")
