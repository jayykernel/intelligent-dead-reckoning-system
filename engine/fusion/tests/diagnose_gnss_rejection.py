"""
engine/fusion/tests/diagnose_gnss_rejection.py

Detailed diagnostic script to inspect raw NIS values, DOF, threshold,
innovation y, R covariance, and S covariance matrices during normal driving.
"""

import os
import sys
import numpy as np

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

def run_diagnostics():
    raw_root = "data/raw"
    driver = "S (Driver A)"
    session = "S4"

    s_df, v_df = load_iovnbd_session(raw_root, driver, session)
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    # Calibrate
    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

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

    # Let's inspect normal driving window: steps 50 to 150 (t = 5.0s to 15.0s) or steps 400 to 700 (t = 40s to 70s)
    # Let's print speeds and stats
    print("Finding normal driving windows in S4:")
    for t_start in [10, 50, 100, 300, 400, 500]:
        t_end = t_start + 100
        mean_speed = np.mean(speed[t_start:t_end])
        print(f"  Window [{t_start}:{t_end}] ({t_start*0.1:.1f}s - {t_end*0.1:.1f}s): Mean Speed = {mean_speed:.2f} m/s ({mean_speed*3.6:.1f} km/h)")

    print("\n--- Detailed step-by-step diagnostic on Steps 350 to 450 (t=35.0s to 45.0s) ---")

    # We will log the detailed update matrices at selected steps
    detailed_samples = [350, 360, 370, 380, 390, 400, 410, 420, 430]

    for i in range(1, 500):
        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])
        m_raw = mag[i] if mag is not None else None

        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=m_raw,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=True,
            timestamp=i*0.1
        )

        if i in detailed_samples:
            print(f"\n================ STEP {i} (t = {i*0.1:.1f}s) ================")
            print(f"GT Speed: {speed[i]:.2f} m/s | GT Heading: {gt_heading[i]:.2f} deg")
            print(f"EKF Pos: {fusion.ekf.p} | GT Pos: {pos_enu}")
            print(f"EKF Vel: {fusion.ekf.v} | GT Vel: {v_enu}")
            r, p, y = fusion.ekf.get_euler_angles_deg()
            print(f"EKF Euler (Roll, Pitch, Yaw): [{r:.2f}, {p:.2f}, {y:.2f}] deg")
            print(f"Gyro Veh (corr): {fusion.calib.apply(acc[i], gyro[i])[1] - fusion.ekf.b_g}")
            print(f"Accel Veh (corr): {fusion.calib.apply(acc[i], gyro[i])[0] - fusion.ekf.b_a}")

            # Print recent NIS history for this timestamp
            recent_nis = [entry for entry in fusion.ekf.nis_history if abs(entry["timestamp"] - i*0.1) < 1e-4]
            for entry in recent_nis:
                print(f"\n  Measurement Type: {entry['type']} (DOF = {entry['dof']})")
                print(f"    Passed: {entry['passed']} | NIS: {entry['nis']:.4f} | Threshold: {entry['threshold']:.4f}")
                print(f"    Innovation y: {entry['innovation']}")

if __name__ == "__main__":
    run_diagnostics()
