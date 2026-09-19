"""
engine/fusion/tests/trace_gnss_rejections.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

def trace():
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

    for i in range(1, 400):
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

        # Check if GNSS_POS or GNSS_VEL was rejected
        recent = [entry for entry in fusion.ekf.nis_history if abs(entry["timestamp"] - i*0.1) < 1e-4]
        gnss_pos_failed = any(e["type"] == "GNSS_POS" and not e["passed"] for e in recent)
        gnss_vel_failed = any(e["type"] == "GNSS_VEL" and not e["passed"] for e in recent)

        if gnss_pos_failed or gnss_vel_failed or (i % 20 == 0):
            print(f"Step {i:3d} (t={i*0.1:.1f}s): Speed={speed[i]:.2f} m/s | Pos Err={np.linalg.norm(fusion.ekf.p[:2] - pos_enu[:2]):.2f}m")
            for e in recent:
                if "GNSS" in e["type"]:
                    print(f"   [{e['type']}] passed={e['passed']}, NIS={e['nis']:.3f} (thresh={e['threshold']:.2f})")

if __name__ == "__main__":
    trace()
