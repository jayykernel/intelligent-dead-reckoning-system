
import os
import sys
import pandas as pd
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")

print("S_df first 5 date strings:")
print(s_df['DATE (YYYY-MO-DD HH-MI-SS_SSS)'].head())

print("V_df first 5 Time Since Start of Day:")
print(v_df['Time Since Start of Day (seconds)'].head())

# Convert S_df date to seconds since start of day
first_date_str = s_df['DATE (YYYY-MO-DD HH-MI-SS_SSS)'].iloc[0]
print("First date str:", first_date_str)
# Format is e.g. 2017-06-29 14-23-10_123 or similar
time_part = first_date_str.split()[1]
parts = time_part.replace(':', '-').replace('_', '-').split('-')
h, m, s, ms = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
s_sec_of_day = h * 3600 + m * 60 + s + ms / 1000.0

v_sec_of_day = v_df['Time Since Start of Day (seconds)'].iloc[0]

print(f"S start sec of day: {s_sec_of_day}")
print(f"V start sec of day: {v_sec_of_day}")
print(f"Time offset difference: {s_sec_of_day - v_sec_of_day:.3f} seconds")
