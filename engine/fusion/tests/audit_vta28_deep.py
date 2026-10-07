import sys, os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

session_config = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}
res = evaluate_dead_reckoning_session(session_config)

s_df, v_df = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta28")
dt = 0.1
synced = preprocess_session(s_df, v_df, target_dt=dt)

outage_start = res["outage_start"]
outage_end = res["outage_end"]

results = res["results"]
print(f"Total steps: {len(results)}")
print(f"Outage: {outage_start} to {outage_end}")

# 1. Check speed_scale prior to outage
pre_scales = [r.get("speed_scale", 1.0) for r in results[:outage_start]]
print(f"Speed scale initial: {pre_scales[0]:.4f}, at outage: {pre_scales[-1]:.4f}, min: {min(pre_scales):.4f}, max: {max(pre_scales):.4f}")

# 2. Check AI speed vs GNSS speed prior to outage
gnss_speeds = synced["gt_speed"].values[:outage_start]
ai_speeds = [r.get("ai_speed", 0.0) for r in results[:outage_start]]
ratios = []
for g, a in zip(gnss_speeds, ai_speeds):
    if g > 2.0 and a > 1.0:
        ratios.append(g / a)
if len(ratios) > 0:
    print(f"Pre-outage GT/AI speed ratio: mean={np.mean(ratios):.3f}, median={np.median(ratios):.3f}, min={np.min(ratios):.3f}, max={np.max(ratios):.3f}")
else:
    print("No valid speed ratios pre-outage")

# 3. Check for NaNs or Infs
nan_in_est = np.isnan(res["est_traj"]).any()
nan_in_cov = any(np.isnan(r.get("cov_2d", np.zeros((2,2)))).any() for r in results)
print(f"NaNs in est_traj: {nan_in_est}, NaNs in cov: {nan_in_cov}")

# 4. Check why final error is high in outage
gt_traj = res["gt_traj"]
est_traj = res["est_traj"]

errors_outage = [np.linalg.norm(est_traj[i] - gt_traj[i]) for i in range(outage_start, outage_end)]
print(f"Outage error start: {errors_outage[0]:.2f}m, mid: {errors_outage[len(errors_outage)//2]:.2f}m, end: {errors_outage[-1]:.2f}m")

# Check heading error during outage
outage_ekf_yaws = []
for i in range(outage_start, outage_end):
    # we can reconstruct yaw from results or check
    pass

# Compare distance travelled GT vs Est
gt_dists = np.sum(np.sqrt(np.diff(gt_traj[outage_start:outage_end, 0])**2 + np.diff(gt_traj[outage_start:outage_end, 1])**2))
est_dists = np.sum(np.sqrt(np.diff(est_traj[outage_start:outage_end, 0])**2 + np.diff(est_traj[outage_start:outage_end, 1])**2))
print(f"Distance travelled in outage: GT={gt_dists:.2f}m, Est={est_dists:.2f}m (ratio={est_dists/gt_dists:.3f})")

