import numpy as np
from eval.run_full_benchmark import load_iovnbd_session, preprocess_session, evaluate_dead_reckoning_session

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}

s_df, v_df = load_iovnbd_session("data/raw", cfg["driver"], cfg["session"])
synced = preprocess_session(s_df, v_df, target_dt=0.1)
gt_speed = synced["gt_speed"].values

res = evaluate_dead_reckoning_session(cfg)
outage_start = res["outage_start"]
outage_end = res["outage_end"]
results = res["results"]

ai_speeds = np.array([r["ai_speed"] for r in results])

# Pre-outage data
pre_ai = ai_speeds[:outage_start-1]
pre_gt = gt_speed[1:outage_start]

# Filter moving segments (speed > 1.0)
mask = (pre_gt > 1.0) & (pre_ai > 1.0)
p_ai = pre_ai[mask]
p_gt = pre_gt[mask]

# 1. Simple scale factor:
k_opt = np.sum(p_ai * p_gt) / np.sum(p_ai**2)

# 2. Affine fit: gt = a * ai + b
A = np.column_stack([p_ai, np.ones_like(p_ai)])
a_opt, b_opt = np.linalg.lstsq(A, p_gt, rcond=None)[0]

print(f"Optimal scale factor k: {k_opt:.3f}")
print(f"Optimal affine: a={a_opt:.3f}, b={b_opt:.3f}")

# Test on outage data
out_ai = ai_speeds[outage_start-1:outage_end]
out_gt = gt_speed[outage_start:outage_end+1]

pred_scale = out_ai * k_opt
pred_affine = a_opt * out_ai + b_opt

err_scale = np.mean(np.abs(pred_scale - out_gt))
err_affine = np.mean(np.abs(pred_affine - out_gt))
dist_scale = np.sum(pred_scale) * 0.1
dist_affine = np.sum(pred_affine) * 0.1
dist_gt = np.sum(out_gt) * 0.1

print(f"\nOutage Ground Truth Distance: {dist_gt:.2f} m")
print(f"Pred Scale Distance:          {dist_scale:.2f} m (MAE: {err_scale:.2f} m/s)")
print(f"Pred Affine Distance:         {dist_affine:.2f} m (MAE: {err_affine:.2f} m/s)")
