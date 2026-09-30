import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session, load_iovnbd_session, preprocess_session

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}

s_df, v_df = load_iovnbd_session("data/raw", cfg["driver"], cfg["session"])
synced = preprocess_session(s_df, v_df, target_dt=0.1)
gt_speed = synced["gt_speed"].values

res = evaluate_dead_reckoning_session(cfg)

outage_start = res["outage_start"]
outage_end = res["outage_end"]
results = res["results"]

ai_speeds = [r["ai_speed"] for r in results]
speed_scales = [r["speed_scale"] for r in results]

print("Index | Rel_t | GT Speed | Raw AI | Scaled AI | Ratio GT/Scaled")
for sec in range(0, 60, 5):
    idx = outage_start + sec * 10
    gt_s = gt_speed[idx]
    ai_s = ai_speeds[idx-1]
    sc = speed_scales[idx-1]
    scaled_s = ai_s * sc
    print(f"{idx:5d} | {sec:4.1f}s | {gt_s:8.2f} | {ai_s:6.2f} | {scaled_s:9.2f} | {gt_s/max(0.1, scaled_s):15.2f}")
