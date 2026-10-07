import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import evaluate_dead_reckoning_session
from eval.run_full_benchmark import build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta26"}

res = evaluate_dead_reckoning_session(cfg)
outage_start = res["outage_start"]
outage_end = res["outage_end"]

s_df, v_df = load_iovnbd_session("data/raw", cfg["driver"], cfg["session"])
synced = preprocess_session(s_df, v_df, target_dt=0.1)

gt_speed = synced["gt_speed"].values

print("="*40)
print(f"Outage indices: {outage_start} to {outage_end}")

speeds = []
ai_speeds = []
vels_2d = []
for i in range(outage_start-50, outage_start):
    speeds.append(gt_speed[i])
    ai_speeds.append(res['results'][i]['ai_speed'])
    v = res['results'][i]['vel']
    vels_2d.append(np.linalg.norm(v[:2]))

print(f"Pre-outage GT speed median/mean: {np.median(speeds):.2f} / {np.mean(speeds):.2f}")
print(f"Pre-outage AI speed median/mean: {np.median(ai_speeds):.2f} / {np.mean(ai_speeds):.2f}")
print(f"Pre-outage EKF speed median/mean: {np.median(vels_2d):.2f} / {np.mean(vels_2d):.2f}")
print(f"Final speed scale: {res['results'][outage_start-1]['speed_scale']:.3f}")

print("\nDuring Outage AI speeds:")
ai_outage = []
for i in range(outage_start, outage_end):
    ai_outage.append(res['results'][i]['ai_speed'])
    
print(f"Outage AI speed median/mean: {np.median(ai_outage):.2f} / {np.mean(ai_outage):.2f}")

print("\nNaN checks:")
nan_count = 0
for r in res['results']:
    if np.any(np.isnan(r['pos'])) or np.any(np.isnan(r['vel'])) or np.any(np.isnan(r['cov_2d'])):
        nan_count += 1
print(f"Total NaNs in states: {nan_count}")

