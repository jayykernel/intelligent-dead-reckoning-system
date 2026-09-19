"""
engine/fusion/tests/test_mag_raw.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

raw_root = "data/raw"
driver = "S (Driver A)"
session = "S4"

s_df, v_df = load_iovnbd_session(raw_root, driver, session)
synced = preprocess_session(s_df, v_df, target_dt=0.1)

mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

print("Raw magnetometer data (first 20 samples):")
for i in range(min(20, len(mag))):
    m = mag[i]
    norm = np.linalg.norm(m)
    print(f"  {i:3d}: {m} | norm = {norm:.2f} uT")

print(f"\nMin norm: {np.min([np.linalg.norm(m) for m in mag]):.2f} uT")
print(f"Max norm: {np.max([np.linalg.norm(m) for m in mag]):.2f} uT")
print(f"Mean norm: {np.mean([np.linalg.norm(m) for m in mag]):.2f} uT")
print(f"Std norm: {np.std([np.linalg.norm(m) for m in mag]):.2f} uT")