from training.data_loader import load_two_wheeler_session
import numpy as np
df = load_two_wheeler_session("data/raw/two_wheeler", "session1")
print(f"Phone Lat: {df['phone_lat'].iloc[0]}, Lon: {df['phone_lon'].iloc[0]}")
print(f"GT Lat:    {df['gt_lat'].iloc[0]}, Lon: {df['gt_lon'].iloc[0]}")
print(f"Diff Lat:  {df['gt_lat'].iloc[0] - df['phone_lat'].iloc[0]}, Lon: {df['gt_lon'].iloc[0] - df['phone_lon'].iloc[0]}")
