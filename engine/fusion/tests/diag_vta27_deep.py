import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, r"C:\dev\dead reckoning proto")

from eval.run_full_benchmark import evaluate_dead_reckoning_session

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})

start = res["outage_start"]
end = res["outage_end"]

gt = res["gt_traj"]
est = res["est_traj"]

outage_gt = gt[start:end]
outage_est = est[start:end]

errors = np.linalg.norm(outage_est - outage_gt, axis=1)

print(f"Error at outage start: {errors[0]:.2f}m")
print(f"Error at outage end: {errors[-1]:.2f}m")
print(f"Max error: {np.max(errors):.2f}m")

# Check heading, speed, biases during outage
results = res["results"]
outage_results = results[start:end]

headings = [r["euler_deg"][2] for r in outage_results]
speeds = [np.linalg.norm(r["vel"]) for r in outage_results]
ai_speeds = [r["ai_speed"] for r in outage_results]
speed_scales = [r["speed_scale"] for r in outage_results]

# True speed during outage
gt_diffs = np.linalg.norm(np.diff(outage_gt, axis=0), axis=1) / 0.1 # assuming dt=0.1
print(f"Mean true speed during outage: {np.mean(gt_diffs):.2f} m/s")
print(f"Mean estimated speed during outage: {np.mean(speeds):.2f} m/s")
print(f"Mean AI raw speed during outage: {np.mean(ai_speeds):.2f} m/s")
print(f"Pre-computed speed scale: {speed_scales[0]:.3f}")

# Look at headings
# Calculate true trajectory heading
gt_headings = np.arctan2(np.diff(outage_gt[:, 0]), np.diff(outage_gt[:, 1])) * 180 / np.pi
print(f"Mean true heading: {np.mean(gt_headings):.2f} deg, std: {np.std(gt_headings):.2f} deg")
print(f"Mean estimated heading: {np.mean(headings):.2f} deg, std: {np.std(headings):.2f} deg")
print(f"Heading error (est - gt) mean: {np.mean(headings[:-1] - gt_headings):.2f} deg")

