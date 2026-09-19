"""
engine/fusion/tests/test_mag_formula.py

Systematically check magnetometer heading formula against GT.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
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

print("=== Checking R_veh_to_nav definition ===")
psi = 0.0
R_test = np.array([
    [np.cos(psi), np.sin(psi), 0.0],
    [-np.sin(psi), np.cos(psi), 0.0],
    [0.0, 0.0, 1.0]
])
print(f"At heading 0 North: R[0,1] = {R_test[0,1]:.3f}, R[1,1] = {R_test[1,1]:.3f}")

print("\n=== Checking magnetometer heading formula ===")

for i in range(0, min(10, len(mag))):
    m_raw = mag[i]
    mag_veh = m_raw @ calib.R_phone_to_veh.T

    psi_gt = np.radians(gt_heading[i])
    R_veh_to_nav = np.array([
        [np.cos(psi_gt), np.sin(psi_gt), 0.0],
        [-np.sin(psi_gt), np.cos(psi_gt), 0.0],
        [0.0, 0.0, 1.0]
    ])

    B_nav = R_veh_to_nav @ mag_veh
    h1 = np.degrees(np.arctan2(B_nav[1], B_nav[0]))  # atan2(N, E) -> 0° = East
    h2 = np.degrees(np.arctan2(B_nav[0], B_nav[1]))  # atan2(E, N) -> 0° = North
    # Correct formula for 0=North: B_East, B_North. Heading = atan2(B_East, B_North)
    h3 = np.degrees(np.arctan2(B_nav[0], B_nav[1]))

    print(f"{i:5d} | GT={gt_heading[i]:6.1f}° | N={B_nav[1]:7.2f}, E={B_nav[0]:7.2f} | atan2(N,E)={h1:7.1f} | atan2(E,N)={h2:7.1f}")
