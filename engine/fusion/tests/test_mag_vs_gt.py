"""
engine/fusion/tests/test_mag_vs_gt.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine
from engine.fusion.mag_gate import MagnetometerGate

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

# Run just the first 100 steps to see mag vs GT heading
for i in range(1, 100):
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])
    m_raw = mag[i] if mag is not None else None

    # Calib
    acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])

    # Predict
    fusion.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=0.1)

    # Compute mag yaw
    R_veh_to_nav = fusion.ekf.quat_to_rot(fusion.ekf.q)
    mag_veh = m_raw @ fusion.calib.R_phone_to_veh.T
    B_nav = R_veh_to_nav @ mag_veh
    mag_yaw = np.arctan2(B_nav[1], B_nav[0])

    # Compare with GT heading
    gt_yaw = np.radians(gt_heading[i])
    diff = (mag_yaw - gt_yaw + np.pi) % (2*np.pi) - np.pi

    if i % 10 == 0:
        print(f"Step {i:3d}: GT={np.degrees(gt_yaw):6.1f}deg, Mag={np.degrees(mag_yaw):6.1f}deg, Diff={np.degrees(diff):6.1f}deg, Speed={speed[i]:.2f}m/s")

# Now check what happens when we use the mag_gate
print("\n--- Using mag_gate ---")
mag_gate = MagnetometerGate()
for i in range(1, 100):
    acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])
    fusion.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=0.1)
    R_veh_to_nav = fusion.ekf.quat_to_rot(fusion.ekf.q)
    mag_veh = mag[i] @ fusion.calib.R_phone_to_veh.T
    is_clean, mag_yaw, info = mag_gate.process_measurement(mag_veh, R_veh_to_nav)
    gt_yaw = np.radians(gt_heading[i])
    if is_clean:
        diff = (mag_yaw - gt_yaw + np.pi) % (2*np.pi) - np.pi
        print(f"Step {i:3d}: GT={np.degrees(gt_yaw):6.1f}deg, Mag={np.degrees(mag_yaw):6.1f}deg, Diff={np.degrees(diff):6.1f}deg, Speed={speed[i]:.2f}m/s, Norm={info['mag_norm']:.1f}")
    else:
        print(f"Step {i:3d}: DISTURBED - {info['reason']}, Norm={info['mag_norm']:.1f}")