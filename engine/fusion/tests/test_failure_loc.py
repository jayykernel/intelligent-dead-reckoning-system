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

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
gt_heading = synced["gt_heading"].values
gt_heading[np.isnan(gt_heading)] = 164.2
p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
h_rad = np.radians(gt_heading[0])
v0 = np.array([speed[0] * np.sin(h_rad), speed[0] * np.cos(h_rad), 0.0])

c = 2.0
fusion = GNSSINSFusionEngine(dt=0.1, enable_ai=False)
fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

for i in range(1, len(synced)):
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
    h_rad_i = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad_i), speed[i] * np.cos(h_rad_i), 0.0])
    
    acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])
    gyro_mag = np.linalg.norm(gyro_veh - fusion.ekf.b_g)
    adaptive_sigma = fusion.ekf.sigma_gyro + c * gyro_mag
    
    fusion.ekf.predict(acc_veh, gyro_veh, dt=0.1)
    fusion.ekf.P[6:9, 6:9] += np.eye(3) * (adaptive_sigma * 0.1)**2
    
    p_pass, _, _ = fusion.ekf.update_gnss_position(pos_enu, sigma_pos=5.0, alpha=0.01, timestamp=i*0.1)
    v_pass, _, _ = fusion.ekf.update_gnss_velocity(v_enu, sigma_vel=0.5, alpha=0.01, timestamp=i*0.1)
    
    speed_2d = np.linalg.norm(v_enu[:2])
    if speed_2d >= 1.5:
        sigma_heading = np.radians(3.0)
    else:
        sigma_heading = np.radians(3.0 + 7.0 * (1.5 - speed_2d) / 1.2)
    cog = float(np.arctan2(v_enu[0], v_enu[1]))
    h_pass, _, _ = fusion.ekf.update_heading(cog, sigma_heading=sigma_heading, alpha=0.01, timestamp=i*0.1, source="GNSS_HEADING")
    
    r, p, y = fusion.ekf.get_euler_angles_deg()
    err = np.abs((y - gt_heading[i] + 180) % 360 - 180)
    if not p_pass or not v_pass or not h_pass or err > 30:
        print(f"Step {i:04d} | Speed={speed[i]:.2f} | Err={err:5.1f} | P_pass={p_pass} | V_pass={v_pass} | H_pass={h_pass}")
        if not p_pass:
            # stop once position fails
            break
