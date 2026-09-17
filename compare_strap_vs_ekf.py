import numpy as np
import os, sys
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.strapdown import StrapdownINS
from engine.fusion.ekf import ErrorStateEKF

s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = synced["gt_speed"].values
speed = np.nan_to_num(speed, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, _ = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
gt_heading = synced["gt_heading"].values
if np.any(np.isnan(gt_heading)):
    gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

p0 = np.array([e_gt[0], n_gt[0], 0.0])
heading_rad = np.radians(gt_heading[0])
v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

strap = StrapdownINS()
strap.initialize_from_gravity_and_heading(p0, v0, acc[0], gt_heading[0])

# Transform IMU with calibrator
acc_v = np.zeros_like(acc)
gyro_v = np.zeros_like(gyro)
for k in range(len(acc)):
    acc_v[k], gyro_v[k] = calib.apply(acc[k], gyro[k])

ekf = ErrorStateEKF(dt=0.1)
ekf.set_initial_state(p0, v0, strap.quat, calib.accel_bias, calib.gyro_bias)

print("Starting comparison for 100 steps...")
for k in range(1, 100):
    p_s, v_s, q_s = strap.step(acc_v[k] - calib.accel_bias, gyro_v[k] - calib.gyro_bias, 0.1)
    ekf.predict(acc_v[k], gyro_v[k], dt=0.1)
    
    pos_diff = np.linalg.norm(p_s - ekf.p)
    vel_diff = np.linalg.norm(v_s - ekf.v)
    quat_diff = np.linalg.norm(q_s - ekf.q)
    if k % 20 == 0 or pos_diff > 1e-4:
        print(f"Step {k}: pos_diff={pos_diff:.6f}, vel_diff={vel_diff:.6f}, quat_diff={quat_diff:.6f}")
        if pos_diff > 1e-4:
            break
