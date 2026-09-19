import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine

raw_root = "data/raw"

sessions = [
    ("car", "S (Driver A)", "S4"),
    ("car", "S (Driver A)", "S1"),
    ("tw", "two_wheeler", "session1"),
    ("tw", "two_wheeler", "session2"),
]

for dtype, driver, session in sessions:
    print(f"--- Evaluating {dtype} {session} ---")
    if dtype == "car":
        s_df, v_df = load_iovnbd_session(raw_root, driver, session)
        synced = preprocess_session(s_df, v_df, target_dt=0.1)
    else:
        synced = load_two_wheeler_session(os.path.join(raw_root, "two_wheeler"), session)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    if mag is None:
        print("  No magnetometer data available.")
        continue

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

    diffs = []
    # Test on a window where speed > 2.0 m/s to ensure GT heading is solid
    for i in range(min(500, len(synced))):
        if speed[i] < 2.0:
            continue

        # Vehicle to Nav
        psi_gt = np.radians(gt_heading[i])
        R_veh_to_nav = np.array([
            [np.cos(psi_gt), np.sin(psi_gt), 0.0],
            [-np.sin(psi_gt), np.cos(psi_gt), 0.0],
            [0.0, 0.0, 1.0]
        ])

        m_veh = mag[i] @ calib.R_phone_to_veh.T
        # B_nav = R_veh_to_nav @ m_veh
        # Geographic heading from mag
        # B_nav = [B_East, B_North, B_Up]
        # In ENU: Geographic Heading = atan2(B_East, B_North)
        B_nav = R_veh_to_nav @ m_veh

        # Geographic yaw in Nav frame: atan2(East, North)
        h_mag = np.degrees(np.arctan2(B_nav[0], B_nav[1]))
        diff = (h_mag - gt_heading[i] + 180) % 360 - 180
        diffs.append(diff)

    if diffs:
        print(f"  Mean heading difference (Mag - GT): {np.mean(diffs):.2f} deg, Std: {np.std(diffs):.2f} deg")
    else:
        print("  Not enough moving samples to calculate diff.")
