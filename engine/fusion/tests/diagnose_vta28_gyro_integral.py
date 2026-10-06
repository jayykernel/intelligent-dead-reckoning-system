import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from training.data_loader import load_iovnbd_session, preprocess_session

raw_root = "data/raw"
driver = "Vta (Driver E)"
session_name = "Vta28"
dt = 0.1

s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
synced = preprocess_session(s_df, v_df, target_dt=dt)

gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
gt_heading = synced["gt_heading"].values

i_start = int(175.0 / dt)
i_end = int(186.0 / dt)

int_x = np.sum(gyro[i_start:i_end, 0]) * dt
int_y = np.sum(gyro[i_start:i_end, 1]) * dt
int_z = np.sum(gyro[i_start:i_end, 2]) * dt

print(f"Integral (deg):")
print(f"  X: {np.degrees(int_x):.1f}")
print(f"  Y: {np.degrees(int_y):.1f}")
print(f"  Z: {np.degrees(int_z):.1f}")
print(f"GT Heading change (deg): {gt_heading[i_end] - gt_heading[i_start]:.1f}")
print(f"GT start: {gt_heading[i_start]:.1f}, GT end: {gt_heading[i_end]:.1f}")
