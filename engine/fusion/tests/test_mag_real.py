import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine

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

gt_heading = synced["gt_heading"].values
gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

print("Testing magnetometer heading in PHONE frame vs VEHICLE frame")
print("Step | GT(°) | mag_raw | mag_veh | heading_raw | heading_veh")

for i in range(0, min(10, len(mag))):
    psi_gt = np.radians(gt_heading[i])
    R_veh_to_nav = np.array([
        [np.cos(psi_gt), np.sin(psi_gt), 0.0],
        [-np.sin(psi_gt), np.cos(psi_gt), 0.0],
        [0.0, 0.0, 1.0]
    ])

    m_raw = mag[i]
    m_veh = m_raw @ calib.R_phone_to_veh.T

    # Direct heading in phone frame
    B_nav_raw = R_veh_to_nav @ m_raw
    B_nav_veh = R_veh_to_nav @ m_veh

    h_raw = np.degrees(np.arctan2(B_nav_raw[1], B_nav_raw[0]))  # atan2(N, E)
    h_veh = np.degrees(np.arctan2(B_nav_veh[1], B_nav_veh[0]))  # atan2(N, E)

    print(f"{i:4d} | {gt_heading[i]:6.1f} | [{m_raw[0]:6.2f}, {m_raw[1]:6.2f}, {m_raw[2]:6.2f}] | "
          f"[{m_veh[0]:6.2f}, {m_veh[1]:6.2f}, {m_veh[2]:6.2f}] | {h_raw:8.1f}° | {h_veh:8.1f}°")

print("\nThe calibration rotation transforms [-17.81, -27.25, 35.75] → [31.29, 6.81, 36.22]")
print("This is a ~90° rotation, which would cause ~50° heading offset.")

print("\nActually, let's check the calibration transformation expectation:")
print("If phone is mounted arbitrarily, R_phone_to_veh rotates phone axes to align with vehicle (Right, Forward, Up).")
print("But for the magnetometer, we want to compute heading in the VEHICLE frame after applying this rotation.")
print("The 50° offset suggests either:")
print("  1. Calibration is wrong (unlikely)")
print("  2. magnetometer reading is already in vehicle frame?")
print("  3. The atan2 formula is wrong (should be atan2(E, N) not atan2(N, E))")

# Try atan2(E, N) instead
print("\n--- Using atan2(E, N) for heading (0° = North) ---")
for i in range(0, 3):
    psi_gt = np.radians(gt_heading[i])
    R_veh_to_nav = np.array([
        [np.cos(psi_gt), np.sin(psi_gt), 0.0],
        [-np.sin(psi_gt), np.cos(psi_gt), 0.0],
        [0.0, 0.0, 1.0]
    ])
    m_veh = mag[i] @ calib.R_phone_to_veh.T
    B_nav = R_veh_to_nav @ m_veh
    h = np.degrees(np.arctan2(B_nav[0], B_nav[1]))  # atan2(E, N)
    diff = (h - gt_heading[i] + 180) % 360 - 180
    print(f"Step {i}: GT={gt_heading[i]:.1f}°, mag={h:.1f}°, diff={diff:.1f}°")
