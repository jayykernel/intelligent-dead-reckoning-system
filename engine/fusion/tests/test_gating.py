import numpy as np
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__))))
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)
fusion = GNSSINSFusionEngine(dt=0.1, enable_ai=False)

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
gt_heading = synced["gt_heading"].values
p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
gt_heading[np.isnan(gt_heading)] = 164.2 # hacky interp
h_rad = np.radians(gt_heading[0])
v0 = np.array([speed[0] * np.sin(h_rad), speed[0] * np.cos(h_rad), 0.0])

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

print("Step | Yaw | GT_H | H_Pass | H_NIS | Thresh | V_Pass")
for i in range(1, 230):
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
    h_rad_i = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad_i), speed[i] * np.cos(h_rad_i), 0.0])
    
    res = fusion.step(acc[i], gyro[i], None, pos_enu, v_enu, True, i*0.1)
    
    # get last heading NIS
    h_nis = 0
    h_pass = False
    h_thresh = 0
    for h in reversed(fusion.ekf.nis_history):
        if h["timestamp"] == i*0.1 and h["type"] == "GNSS_HEADING":
            h_nis = h["nis"]
            h_pass = h["passed"]
            h_thresh = h["threshold"]
            break
            
    r, p, y = fusion.ekf.get_euler_angles_deg()
    if i >= 170 and i <= 210:
        print(f"{i:03d}  | {y:5.1f} | {gt_heading[i]:5.1f} | {h_pass!s:6} | {h_nis:6.1f} | {h_thresh:5.1f} | {res['gnss_vel_passed']}")

