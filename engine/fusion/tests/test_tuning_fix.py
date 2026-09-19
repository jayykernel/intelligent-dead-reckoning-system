"""
engine/fusion/tests/test_tuning_fix.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

def test():
    raw_root = "data/raw"
    driver = "S (Driver A)"
    session = "S4"

    s_df, v_df = load_iovnbd_session(raw_root, driver, session)
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

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

    # Let's test with sigma_vel = 1.0 m/s
    N = len(synced)
    outage_start = int(N * 0.4)
    outage_end = outage_start + int(60.0 / 0.1)
    outage_end = min(outage_end, N - int(10.0 / 0.1))

    for i in range(1, N):
        in_outage = (outage_start <= i <= outage_end)
        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
        m_raw = mag[i] if mag is not None else None

        # Step manually with sigma_vel = 1.0 m/s in update_gnss_velocity
        # 1. Calib
        acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])

        # 2. Predict
        fusion.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=0.1)

        # 3. NHC/ZUPT
        R_veh_to_nav = fusion.ekf.quat_to_rot(fusion.ekf.q)
        v_veh = R_veh_to_nav.T @ fusion.ekf.v
        is_stopped = fusion.constrained_ins._is_stopped(acc_veh, gyro_veh, v_veh)
        if is_stopped:
            fusion.ekf.update_zupt(sigma_zupt=0.1, timestamp=i*0.1)
        else:
            fusion.ekf.update_nhc(vehicle_type="car", sigma_nhc_x=0.2, sigma_nhc_z=0.2, timestamp=i*0.1)

        # 4. Mag
        if m_raw is not None:
            mag_veh = m_raw @ fusion.calib.R_phone_to_veh.T
            is_clean, mag_yaw, _ = fusion.mag_gate.process_measurement(mag_veh, R_veh_to_nav)
            if is_clean and mag_yaw is not None:
                fusion.ekf.update_heading(heading_rad=mag_yaw, sigma_heading=np.radians(8.0), timestamp=i*0.1, source="MAG_HEADING")

        # 5. GNSS
        if not in_outage and pos_enu is not None:
            fusion.ekf.update_gnss_position(p_gnss_enu=pos_enu, sigma_pos=5.0, timestamp=i*0.1)
            if v_enu is not None:
                fusion.ekf.update_gnss_velocity(v_gnss_enu=v_enu, sigma_vel=1.0, timestamp=i*0.1)
                speed_2d = np.linalg.norm(v_enu[:2])
                if speed_2d >= 1.0:
                    cog_h = float(np.arctan2(v_enu[0], v_enu[1]))
                    fusion.ekf.update_heading(heading_rad=cog_h, sigma_heading=np.radians(5.0), timestamp=i*0.1, source="GNSS_HEADING")

    # Evaluate outage drift
    pos_est = np.array(fusion.trajectory_pos)
    outage_gt = np.column_stack([e_gt[outage_start:outage_end], n_gt[outage_start:outage_end]])
    outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt[:, 0])**2 + np.diff(outage_gt[:, 1])**2)))
    final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))
    drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

    gnss_updates = [n for n in fusion.ekf.nis_history if n["type"] == "GNSS_POS"]
    passed_gnss = sum(1 for n in gnss_updates if n["passed"])
    rejected_gnss = len(gnss_updates) - passed_gnss

    print(f"Results with sigma_vel=1.0 m/s:")
    print(f"  Outage Dist: {outage_dist:.2f} m")
    print(f"  Final Error: {final_err:.2f} m")
    print(f"  Drift %:     {drift_pct:.2f}%")
    print(f"  GNSS POS (Passed/Rejected): {passed_gnss} / {rejected_gnss}")

if __name__ == "__main__":
    test()
