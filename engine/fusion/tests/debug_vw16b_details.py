import os
import sys
import numpy as np
import pandas as pd

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

raw_root = "data/raw"
s_df, v_df = load_iovnbd_session(raw_root, "Vw (Driver E)", "Vw16b")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

print("Number of points:", len(synced))
print("Duration (s):", len(synced) * 0.1)

# Check heading, gyro, speed, accel
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
gt_heading = synced["gt_heading"].values

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)
print("Calib R_phone_to_veh:\n", calib.R_phone_to_veh)
print("Calib gyro_bias:", calib.gyro_bias)
print("Calib accel_bias:", calib.accel_bias)

# Check gyro integration vs GT heading
dt = 0.1
acc_veh, gyro_veh = calib.apply(acc, gyro)
integrated_yaw = [gt_heading[0]]
for g in gyro_veh:
    # yaw rate is in vehicle frame z? or what?
    pass

# Check what happens during pre-outage (0 to 45s)
print("Pre-outage duration: 45s (450 steps)")
print("Speed stats in pre-outage: mean=", np.mean(speed[:450]), "max=", np.max(speed[:450]))
print("Heading range in pre-outage: min=", np.min(gt_heading[:450]), "max=", np.max(gt_heading[:450]))
print("Heading change across entire session: min=", np.min(gt_heading), "max=", np.max(gt_heading))

