import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session

# Load raw data
s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")

# Parse absolute times
date_str = s_df['DATE (YYYY-MO-DD HH-MI-SS_SSS)'].iloc[0]
time_part = date_str.split()[1].replace(':', '-').replace('_', '-')
h, m, s, ms = map(int, time_part.split('-'))
s_start_sec = h * 3600 + m * 60 + s + ms / 1000.0

s_time = s_start_sec + (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0
v_time = v_df['Time Since Start of Day (seconds)'].values

t_start = max(s_time.iloc[0], v_time[0])
t_end = min(s_time.iloc[-1], v_time[-1])
dt = 0.1
uniform_time_abs = np.arange(t_start, t_end, dt)

# Interpolate all gyro columns
gyro_yaw = np.interp(uniform_time_abs, s_time, s_df['GYROSCOPE Yaw (rad/s)'])
gyro_pitch = np.interp(uniform_time_abs, s_time, s_df['GYROSCOPE Pitch (rad/s)'])
gyro_roll = np.interp(uniform_time_abs, s_time, s_df['GYROSCOPE Roll (rad/s)'])

# GT yaw rate
unwrapped_h = np.unwrap(np.radians(np.interp(uniform_time_abs, v_time, v_df['Heading (degrees)'])))
gt_yaw_rate = np.diff(unwrapped_h) / dt
gt_yaw_rate = np.append(gt_yaw_rate, gt_yaw_rate[-1])

print("Correlation with GT yaw rate (time-aligned):")
for name, g in [('Yaw', gyro_yaw), ('Pitch', gyro_pitch), ('Roll', gyro_roll)]:
    corr = np.corrcoef(g, gt_yaw_rate)[0, 1]
    print(f"  Gyro {name}: {corr:+.4f}, std={np.std(g):.4f}")

# Also check during moving periods
gt_speed = np.interp(uniform_time_abs, v_time, v_df['Velocity (km/hr)']) / 3.6
moving = gt_speed > 2.0
print(f"\nMoving samples: {np.sum(moving)} / {len(moving)}")
for name, g in [('Yaw', gyro_yaw), ('Pitch', gyro_pitch), ('Roll', gyro_roll)]:
    corr = np.corrcoef(g[moving], gt_yaw_rate[moving])[0, 1]
    print(f"  Gyro {name} (moving): {corr:+.4f}")