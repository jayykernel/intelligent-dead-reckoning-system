import numpy as np
import os
from eval.run_full_benchmark import evaluate_dead_reckoning_session

cfg = {"category": "two_wheeler", "session": "session2"}
res = evaluate_dead_reckoning_session(cfg)

outage_start = res["outage_start"]
outage_end = res["outage_end"]
pos_err = res["final_pos_error"]
drift = res["drift_pct"]
dist = res["distance_travelled"]

print(f"\nResults for session2:")
print(f"Drift: {drift:.2f}%")
print(f"Dist: {dist:.2f} m")
print(f"Pos error: {pos_err:.2f} m")

# Look at EKF vs GT positions during outage
results = res["results"]
gt_pos = res["gt_pos"]

ekf_pos = np.array([r["pos"] for r in results])

errs = np.linalg.norm(ekf_pos[outage_start-1:outage_end] - gt_pos[outage_start:outage_end+1], axis=1)
print(f"Max error during outage: {np.max(errs):.2f} m")
print(f"Final error at end of outage: {errs[-1]:.2f} m")

# Let's inspect heading errors
gt_heading = res["gt_heading"]
ekf_headings = [r["euler_deg"][2] for r in results]
heading_errs = [((ekf_headings[i] - gt_heading[i+1] + 180) % 360) - 180 for i in range(outage_start-1, outage_end)]
print(f"Mean heading error in outage: {np.mean(np.abs(heading_errs)):.2f} deg")
print(f"Final heading error in outage: {heading_errs[-1]:.2f} deg")

