"""
engine/fusion/tests/inspect_matrices.py

Inspect the full R, S, P matrices, DOF, and thresholds for GNSS updates.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

def run():
    raw_root = "data/raw"
    driver = "S (Driver A)"
    session = "S4"

    s_df, v_df = load_iovnbd_session(raw_root, driver, session)
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

    print("=== CALIBRATION PARAMETERS ===")
    print(f"Calibrated: {calib.is_calibrated}")
    print(f"R_phone_to_veh:\n{calib.R_phone_to_veh}")
    print(f"Accel Bias: {calib.accel_bias}")
    print(f"Gyro Bias: {calib.gyro_bias}")

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

    fusion = GNSSINSFusionEngine(dt=0.1, enable_ai=False)
    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    # Inspect normal driving step 350 (speed ~3.2 m/s, straight road)
    for i in range(1, 351):
        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])

        if i == 350:
            print(f"\n================ INSPECTING MATRICES AT STEP 350 ================")
            # 1. Prediction step
            acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])
            print(f"Raw Phone Acc: {acc[i]}")
            print(f"Veh Frame Acc: {acc_veh}")
            print(f"Veh Frame Gyro: {gyro_veh}")
            print(f"EKF Acc Bias b_a: {fusion.ekf.b_a}")
            print(f"EKF Gyro Bias b_g: {fusion.ekf.b_g}")

            # Run predict
            fusion.ekf.predict(acc_veh, gyro_veh, dt=0.1)

            print("\n--- 1. GNSS POSITION UPDATE ---")
            z_pos = pos_enu
            h_pos = fusion.ekf.p
            y_pos = z_pos - h_pos
            H_pos = np.zeros((3, 15))
            H_pos[0:3, 0:3] = np.eye(3)
            R_pos = np.eye(3) * (5.0**2)
            S_pos = H_pos @ fusion.ekf.P @ H_pos.T + R_pos
            nis_pos = float(y_pos.T @ np.linalg.inv(S_pos) @ y_pos)
            print(f"DOF: 3 | Threshold (alpha=0.01): 11.3449 | NIS: {nis_pos:.4f}")
            print(f"Innovation y_pos (m):\n  {y_pos}")
            print(f"R_pos matrix (m^2):\n{R_pos}")
            print(f"S_pos matrix (m^2):\n{S_pos}")
            print(f"P_pos (P[0:3, 0:3]):\n{fusion.ekf.P[0:3, 0:3]}")

            print("\n--- 2. GNSS VELOCITY UPDATE ---")
            z_vel = v_enu
            h_vel = fusion.ekf.v
            y_vel = z_vel - h_vel
            H_vel = np.zeros((3, 15))
            H_vel[0:3, 3:6] = np.eye(3)
            R_vel = np.eye(3) * (0.5**2)
            S_vel = H_vel @ fusion.ekf.P @ H_vel.T + R_vel
            nis_vel = float(y_vel.T @ np.linalg.inv(S_vel) @ y_vel)
            print(f"DOF: 3 | Threshold (alpha=0.01): 11.3449 | NIS: {nis_vel:.4f}")
            print(f"Innovation y_vel (m/s):\n  {y_vel}")
            print(f"R_vel matrix ((m/s)^2):\n{R_vel}")
            print(f"S_vel matrix ((m/s)^2):\n{S_vel}")
            print(f"P_vel (P[3:6, 3:6]):\n{fusion.ekf.P[3:6, 3:6]}")

            print("\n--- 3. GNSS COG HEADING UPDATE ---")
            R_mat = fusion.ekf.quat_to_rot(fusion.ekf.q)
            current_yaw = float(np.arctan2(R_mat[0, 1], R_mat[1, 1]))
            cog_heading = float(np.arctan2(v_enu[0], v_enu[1]))
            z_hdg = np.array([cog_heading])
            h_hdg = np.array([current_yaw])
            y_hdg = (z_hdg - h_hdg + np.pi) % (2 * np.pi) - np.pi
            H_hdg = np.zeros((1, 15))
            H_hdg[0, 8] = -1.0
            sigma_h = np.radians(3.0)
            R_hdg = np.array([[sigma_h**2]])
            S_hdg = H_hdg @ fusion.ekf.P @ H_hdg.T + R_hdg
            nis_hdg = float(y_hdg.T @ np.linalg.inv(S_hdg) @ y_hdg)
            print(f"DOF: 1 | Threshold (alpha=0.01): 6.6349 | NIS: {nis_hdg:.4f}")
            print(f"Current EKF Yaw: {np.degrees(current_yaw):.2f} deg | COG Heading: {np.degrees(cog_heading):.2f} deg")
            print(f"Innovation y_hdg: {y_hdg[0]:.4f} rad ({np.degrees(y_hdg[0]):.2f} deg)")
            print(f"R_hdg: {R_hdg[0, 0]:.6f} rad^2 (sigma = {np.degrees(sigma_h):.2f} deg)")
            print(f"S_hdg: {S_hdg[0, 0]:.6f} rad^2 (sqrt(S) = {np.degrees(np.sqrt(S_hdg[0, 0])):.2f} deg)")
            print(f"P[8, 8] (yaw var): {fusion.ekf.P[8, 8]:.6f} rad^2 (std = {np.degrees(np.sqrt(fusion.ekf.P[8, 8])):.2f} deg)")
            break

        fusion.step(acc[i], gyro[i], None, pos_enu, v_enu, True, i*0.1)

if __name__ == "__main__":
    run()
