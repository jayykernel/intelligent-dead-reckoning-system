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
    print(f"\n--- Testing Formula on {dtype} {session} ---")
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
        print("  No magnetometer data.")
        continue

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

    diffs_formula = []
    for i in range(min(1000, len(synced))):
        if speed[i] < 2.0:
            continue

        # Calibrated magnetometer in vehicle frame
        m_veh = calib.R_phone_to_veh @ mag[i]

        # Heading formula: atan2(-Bx, By)
        h_mag = np.degrees(np.arctan2(-m_veh[0], m_veh[1]))
        diff = (h_mag - gt_heading[i] + 180) % 360 - 180
        diffs_formula.append(diff)

    if diffs_formula:
        print(f"  Formula atan2(-Bx, By): Mean diff = {np.mean(diffs_formula):.2f} deg, Std = {np.std(diffs_formula):.2f} deg")
    else:
        print("  No samples.")
