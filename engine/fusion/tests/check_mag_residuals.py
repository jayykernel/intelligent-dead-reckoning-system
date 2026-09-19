import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.mag_gate import MagnetometerGate

raw_root = "data/raw"

sessions = [
    ("car", "S (Driver A)", "S4"),
    ("car", "S (Driver A)", "S1"),
    ("tw", "two_wheeler", "session1"),
    ("tw", "two_wheeler", "session2"),
]

for dtype, driver, session in sessions:
    print(f"\n--- Checking Tilt-Compensated Mag Gate on {dtype} {session} ---")
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

    gate = MagnetometerGate()
    valid_count = 0
    disturbed_count = 0
    residuals = []

    for i in range(len(synced)):
        psi_gt = np.radians(gt_heading[i])
        # Vehicle forward Y aligned with Heading
        R_veh_to_nav = np.array([
            [np.cos(psi_gt), np.sin(psi_gt), 0.0],
            [-np.sin(psi_gt), np.cos(psi_gt), 0.0],
            [0.0, 0.0, 1.0]
        ])

        mag_veh = calib.R_phone_to_veh @ mag[i]
        is_clean, mag_yaw, debug = gate.process_measurement(mag_veh, R_veh_to_nav)

        if is_clean and mag_yaw is not None:
            valid_count += 1
            # In ENU: If R_veh_to_nav is perfect, B_nav = R_veh_to_nav @ mag_veh
            # B_nav = [B_East, B_North, B_Up].
            # True North is [0, 1, 0].
            # angle should be atan2(B_East, B_North).
            # The residual angle from True North:
            res = float(np.degrees(np.arctan2(debug.get("B_East", 0.0) if "B_East" in debug else np.sin(mag_yaw),
                                              debug.get("B_North", 1.0) if "B_North" in debug else np.cos(mag_yaw))))
            # Let's inspect raw B_nav
            B_nav = R_veh_to_nav @ mag_veh
            angle_from_north = np.degrees(np.arctan2(B_nav[0], B_nav[1]))
            residuals.append(angle_from_north)
        else:
            disturbed_count += 1

    print(f"  Clean: {valid_count}, Disturbed: {disturbed_count}")
    if residuals:
        print(f"  Angle of B_nav from True North: Mean = {np.mean(residuals):.2f} deg, Std = {np.std(residuals):.2f} deg, Median = {np.median(residuals):.2f} deg")
