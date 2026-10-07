
import os
import sys
import pandas as pd
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")

s_speed = s_df['GPS SPEED (Kmh)'].values / 3.6
v_speed = v_df['Velocity (km/hr)'].values / 3.6

s_time_ms = s_df['TIME SINCE START (ms)'].values / 1000.0
v_time_s = (v_df['Time Since Start of Day (seconds)'] - v_df['Time Since Start of Day (seconds)'].iloc[0]).values

print("S speed shape:", s_speed.shape, "V speed shape:", v_speed.shape)
print("S duration:", s_time_ms[-1], "V duration:", v_time_s[-1])

# Test cross correlation of speed
t_grid = np.arange(0, min(s_time_ms[-1], v_time_s[-1]), 0.1)
s_sp_interp = np.interp(t_grid, s_time_ms, s_speed)
v_sp_interp = np.interp(t_grid, v_time_s, v_speed)

corr_0 = np.corrcoef(s_sp_interp, v_sp_interp)[0, 1]
print(f"Speed correlation at 0 offset: {corr_0:.4f}")

# Cross correlation over shifts
lags = np.arange(-300, 300, 1) # -30s to +30s in 0.1s steps
corrs = []
for lag in lags:
    if lag < 0:
        c = np.corrcoef(s_sp_interp[:lag], v_sp_interp[-lag:])[0, 1]
    elif lag > 0:
        c = np.corrcoef(s_sp_interp[lag:], v_sp_interp[:-lag])[0, 1]
    else:
        c = corr_0
    corrs.append(c)

best_lag = lags[np.argmax(corrs)]
print(f"Best lag: {best_lag * 0.1:.2f} seconds with max correlation = {np.max(corrs):.4f}")
