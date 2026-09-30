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

# Align results with gt_speed (results starts from index 1)
# outage_start to outage_end
print("\n--- OUTAGE SPEED ANALYSIS ---")
outage_ai = np.array(ai_speeds[outage_start-1:outage_end])
outage_scale = np.array(speed_scales[outage_start-1:outage_end])
outage_gt = np.array(gt_speed[outage_start:outage_end+1])

print(f"Mean GT speed:        {np.mean(outage_gt):.2f} m/s")
print(f"Mean Raw AI speed:    {np.mean(outage_ai):.2f} m/s")
print(f"Speed scale:          {outage_scale[0]:.3f}")
print(f"Mean Scaled AI speed: {np.mean(outage_ai * outage_scale):.2f} m/s")
print(f"Ratio GT / Raw AI:    {np.mean(outage_gt) / np.mean(outage_ai):.3f}")

# Let's inspect before outage:
pre_ai = np.array(ai_speeds[:outage_start-1])
pre_scale = np.array(speed_scales[:outage_start-1])
pre_gt = np.array(gt_speed[1:outage_start])
print(f"\n--- PRE-OUTAGE SPEED ANALYSIS ---")
print(f"Mean GT speed:        {np.mean(pre_gt):.2f} m/s")
print(f"Mean Raw AI speed:    {np.mean(pre_ai):.2f} m/s")
print(f"Ratio GT / Raw AI:    {np.mean(pre_gt) / np.mean(pre_ai):.3f}")
