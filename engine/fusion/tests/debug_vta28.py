"""
engine/fusion/tests/debug_vta28.py
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.map_matching.road_network import RoadNetwork
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from eval.run_full_benchmark import build_gt_road_network

s_df, v_df = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta28")
dt = 0.1
synced = preprocess_session(s_df, v_df, target_dt=dt)

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
gnss = np.column_stack([e_gt, n_gt])
print("Total GNSS points:", len(gnss))
print("GNSS start:", gnss[0], "end:", gnss[-1])

# Generate road network
road_net = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
print("Total segments:", len(road_net.segments))

# Outage window
N = len(gnss)
outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / dt)

print(f"Outage indices: {outage_start} to {outage_end} (t={outage_start*dt:.1f}s to {outage_end*dt:.1f}s)")
print(f"Start pos: {gnss[outage_start]}, End pos: {gnss[outage_end]}")

# Look at GT trajectory during outage
for i in range(outage_start, outage_end, int(5.0 / dt)):
    pt = gnss[i]
    cands = road_net.find_candidate_segments(pt, radius_m=50.0, max_candidates=5)
    print(f"t={i*dt:.1f}s GT {pt[:2]}: candidate segs = {[(c[0].segment_id, round(c[2], 1), round(c[0].bearing_deg, 1)) for c in cands]}")


