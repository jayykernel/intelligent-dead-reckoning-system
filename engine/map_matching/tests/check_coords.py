import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from training.data_loader import load_iovnbd_session

s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
print("s_df cols:", s_df.columns.tolist()[:10])
print("v_df cols:", v_df.columns.tolist()[:10])
lat = v_df["gt_lat"].dropna().values
lon = v_df["gt_lon"].dropna().values
print(f"S4 Lat Range: [{lat.min():.6f}, {lat.max():.6f}]")
print(f"S4 Lon Range: [{lon.min():.6f}, {lon.max():.6f}]")
