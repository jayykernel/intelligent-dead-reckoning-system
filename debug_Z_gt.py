import numpy as np
import os, sys
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)

print(f"Alt0: {alt0}")
print(f"Alts: {synced['gt_alt'].values[:10]}")
print(f"u_gt: {u_gt[:10]}")
