import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session

# Let's run Vta28 and look at gyro, heading, road segments
cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}
res = evaluate_dead_reckoning_session(cfg)

outage_start = res["outage_start"]
outage_end = res["outage_end"]
gt_traj = res["gt_traj"]
est_traj = res["est_traj"]
results = res["results"]

# Let's inspect step by step from outage_start + 400 to outage_start + 600 (every 10 steps = 1s)
for i in range(outage_start + 400, outage_end, 10):
    t_rel = (i - outage_start) * 0.1
    p_gt = gt_traj[i]
    p_est = est_traj[i]
    euler = results[i-1]["euler_deg"]
    print(f"t={t_rel:4.1f}s | GT: ({p_gt[0]:6.1f}, {p_gt[1]:6.1f}) | EST: ({p_est[0]:6.1f}, {p_est[1]:6.1f}) | Yaw: {euler[2]:6.1f}° | Err: {np.linalg.norm(p_gt-p_est):5.1f}m")
