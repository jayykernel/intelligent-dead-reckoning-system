import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}
res = evaluate_dead_reckoning_session(cfg)

outage_start = res["outage_start"]
outage_end = res["outage_end"]
gt_traj = res["gt_traj"]
est_traj = res["est_traj"]

print("\n--- DETAILED OUTAGE ANALYSIS (Vta28) ---")
for idx in range(outage_start, outage_end + 1, 50): # every 5s
    t_rel = (idx - outage_start) * 0.1
    p_gt = gt_traj[idx]
    p_est = est_traj[idx]
    err = np.linalg.norm(p_gt - p_est)
    d_gt = np.linalg.norm(gt_traj[idx] - gt_traj[outage_start])
    d_est = np.linalg.norm(est_traj[idx] - est_traj[outage_start])
    print(f"t={t_rel:4.1f}s | GT: ({p_gt[0]:7.1f}, {p_gt[1]:7.1f}) | EST: ({p_est[0]:7.1f}, {p_est[1]:7.1f}) | Err: {err:6.2f}m | Dist GT: {d_gt:6.1f}m, Est: {d_est:6.1f}m")
