import sys
import numpy as np
sys.path.insert(0, r"C:\dev\dead reckoning proto")
from eval.run_full_benchmark import evaluate_dead_reckoning_session

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})

start = res["outage_start"]
end = res["outage_end"]
gt = res["gt_traj"]
est = res["est_traj"]

print(f"Time (s) | GT Pos (E, N) | Est Pos (E, N) | Pos Err (m) | GT Heading | Est Heading | Gyro Z (bias)")
for i in range(start, min(start + 600, end), 60):
    t = i * 0.1
    gt_p = gt[i]
    est_p = est[i]
    err = np.linalg.norm(est_p - gt_p)
    r = res["results"][i]
    yaw = r["euler_deg"][2]
    # gt heading
    if i < len(gt) - 1:
        gt_yaw = np.arctan2(gt[i+1, 0] - gt[i, 0], gt[i+1, 1] - gt[i, 1]) * 180 / np.pi
    else:
        gt_yaw = 0.0
    print(f"{t:6.1f}s | ({gt_p[0]:6.1f}, {gt_p[1]:6.1f}) | ({est_p[0]:6.1f}, {est_p[1]:6.1f}) | {err:6.1f}m | {gt_yaw:6.1f} deg | {yaw:6.1f} deg")

