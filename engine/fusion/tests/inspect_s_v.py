
import os
import sys
import pandas as pd
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")

print("s_df shape:", s_df.shape)
print("v_df shape:", v_df.shape)
print("s_df columns:", s_df.columns.tolist())
print(s_df.head())
print(v_df.head())

print("s_df summary statistics for Gyro:")
for col in s_df.columns:
    if 'GYROSCOPE' in col or 'ACCEL' in col:
        print(col, "min:", s_df[col].min(), "max:", s_df[col].max(), "std:", s_df[col].std())
