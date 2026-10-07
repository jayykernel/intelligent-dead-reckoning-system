
import os
import sys
import glob
import pandas as pd
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")
print("S_df columns:", s_df.columns.tolist())
print("V_df columns:", v_df.columns.tolist())

s_time = (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0
v_time = v_df['Time Since Start of Day (seconds)'] - v_df['Time Since Start of Day (seconds)'].iloc[0]
t_grid = np.arange(0, min(s_time.iloc[-1], v_time.iloc[-1]), 0.1)

# GT yaw rate
unwrapped_h = np.unwrap(np.radians(v_df['Heading (degrees)']))
gt_yaw_rate = np.diff(np.interp(t_grid, v_time, unwrapped_h)) / 0.1
gt_yaw_rate = np.append(gt_yaw_rate, gt_yaw_rate[-1])

for c in [x for x in s_df.columns if 'GYROSCOPE' in x]:
    val = np.interp(t_grid, s_time, s_df[c])
    corr = np.corrcoef(val, gt_yaw_rate)[0, 1]
    print(f"Column {c}: std={np.std(val):.4f}, corr with GT yaw rate = {corr:+.4f}")
