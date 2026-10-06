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

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

for i in range(1600, 1900, 10):
    print(f"t={i*dt:5.1f}s | acc: x={acc[i,0]:5.1f}, y={acc[i,1]:5.1f}, z={acc[i,2]:5.1f} | gyro: x={np.degrees(gyro[i,0]):5.1f}, y={np.degrees(gyro[i,1]):5.1f}, z={np.degrees(gyro[i,2]):5.1f} | speed: {speed[i]:.1f}")
