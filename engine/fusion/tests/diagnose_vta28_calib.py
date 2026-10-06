import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine

raw_root = "data/raw"
driver = "Vta (Driver E)"
session_name = "Vta28"
dt = 0.1

s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
synced = preprocess_session(s_df, v_df, target_dt=dt)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

stationary_mask = speed[:1200] < 0.5
stat_acc = acc[:1200][stationary_mask]
g_phone = np.mean(stat_acc, axis=0)

ds = np.gradient(speed[:1200], dt)
accel_mask = ds > 0.5
fwd_acc = acc[:1200][accel_mask] - g_phone
a_fwd_phone = np.mean(fwd_acc, axis=0)

print(f"g_phone: {g_phone}")
print(f"a_fwd_phone: {a_fwd_phone}")
