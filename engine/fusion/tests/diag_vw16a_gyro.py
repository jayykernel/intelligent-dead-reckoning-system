import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import (
    load_iovnbd_session,
    preprocess_session,
    CalibrationEngine
)

s_df, v_df = load_iovnbd_session("data/raw", "Vw (Driver E)", "Vw16a")
synced = preprocess_session(s_df, v_df, target_dt=0.1)
N = len(synced)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
gt_heading = synced["gt_heading"].values
if np.any(np.isnan(gt_heading)):
    gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / 0.1)

print(f"Calibration gyro bias: {calib.gyro_bias}")
print(f"R_phone_to_veh:\n{calib.R_phone_to_veh}")

# Rotate gyro to vehicle frame
gyro_veh = (gyro - calib.gyro_bias) @ calib.R_phone_to_veh.T

# Outage gyro_z vs gt_heading rate
gt_h_unwrapped = np.unwrap(np.radians(gt_heading))
# Note: gt_heading is degrees clockwise from North (Compass). ENU yaw = 90 - compass.
# So d(yaw_enu)/dt = - d(heading_compass)/dt.
gt_yaw_rate = -np.gradient(gt_h_unwrapped, 0.1)

outage_gyro_z = gyro_veh[outage_start:outage_end, 2]
outage_gt_rate = gt_yaw_rate[outage_start:outage_end]

print("\n--- Outage Heading / Gyro comparison ---")
print(f"Mean gyro_z in vehicle frame: {np.mean(outage_gyro_z):.5f} rad/s ({np.degrees(np.mean(outage_gyro_z)):.3f} deg/s)")
print(f"Mean GT yaw rate: {np.mean(outage_gt_rate):.5f} rad/s ({np.degrees(np.mean(outage_gt_rate)):.3f} deg/s)")
print(f"Difference (uncompensated gyro drift/bias rate): {np.degrees(np.mean(outage_gyro_z) - np.mean(outage_gt_rate)):.3f} deg/s")
print(f"Total accumulated uncompensated heading drift over 60s: {np.degrees(np.mean(outage_gyro_z) - np.mean(outage_gt_rate))*60:.2f} deg")

# What about pre-outage GNSS confidence / speed scale learning?
print("\n--- Pre-Outage GNSS / Speed scale details ---")
# Check GPS fixes and speed in pre-outage
for w_sec in [10, 30, 60, 100, 200]:
    start_idx = max(0, outage_start - int(w_sec/0.1))
    w_speed = speed[start_idx:outage_start]
    print(f"Last {w_sec}s pre-outage: mean speed = {np.mean(w_speed):.2f} m/s, min={np.min(w_speed):.2f}, max={np.max(w_speed):.2f}")
