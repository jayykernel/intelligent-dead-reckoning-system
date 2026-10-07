import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath("."))
from training.data_loader import load_iovnbd_session, preprocess_session

driver = "Vtb (Driver E)"
session_name = "Vtb12"
raw_root = "data/raw"
dt = 0.1

s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
synced = preprocess_session(s_df, v_df, target_dt=dt)

N = len(synced)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / dt)
outage_end = min(outage_end, N - int(10.0 / dt))

print(f"Total points: {N}")
print(f"Pre-outage: 0 to {outage_start} ({outage_start*dt:.1f}s)")
print(f"Outage: {outage_start} to {outage_end} ({outage_start*dt:.1f}s - {outage_end*dt:.1f}s, duration {(outage_end-outage_start)*dt:.1f}s)")

pre_acc = acc[:outage_start]
out_acc = acc[outage_start:outage_end]

print("Pre-outage Accel norm mean/std:", np.mean(np.linalg.norm(pre_acc, axis=1)), np.std(np.linalg.norm(pre_acc, axis=1)))
print("Outage Accel norm mean/std:", np.mean(np.linalg.norm(out_acc, axis=1)), np.std(np.linalg.norm(out_acc, axis=1)))

pre_gyro = gyro[:outage_start]
out_gyro = gyro[outage_start:outage_end]

print("Pre-outage Gyro norm mean/std:", np.mean(np.linalg.norm(pre_gyro, axis=1)), np.std(np.linalg.norm(pre_gyro, axis=1)))
print("Outage Gyro norm mean/std:", np.mean(np.linalg.norm(out_gyro, axis=1)), np.std(np.linalg.norm(out_gyro, axis=1)))

