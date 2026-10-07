
import os
import sys
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

print("Speed [:1200] min:", np.min(speed[:1200]), "max:", np.max(speed[:1200]), "mean:", np.mean(speed[:1200]))
print("Number of stationary samples (speed < 0.2) in first 1200:", np.sum(speed[:1200] < 0.2))
print("Number of stationary samples (speed < 0.5) in first 1200:", np.sum(speed[:1200] < 0.5))
print("Number of stationary samples in entire session:", np.sum(speed < 0.2))
