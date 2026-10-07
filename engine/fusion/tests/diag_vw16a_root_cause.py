import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import evaluate_dead_reckoning_session

sess_config = {"category": "car", "driver": "Vw (Driver E)", "session": "Vw16a"}
res = evaluate_dead_reckoning_session(sess_config)

# 1. Check speed scale
print(f"Speed scale: {res.get('speed_scale', 'unknown')}")

# Check AI speeds vs GPS speeds right before outage
traj = res['results']
outage_start_idx = res['outage_start']

# 2. Check heading divergence
heading_pre = []
heading_out = []
heading_gt_pre = []
heading_gt_out = []

for r in traj[outage_start_idx-50:outage_start_idx]:     
    heading_pre.append(r['yaw'])
    if r['gps_fix_valid']:
       heading_gt_pre.append(r['gps_speed_heading'])

for r in traj[outage_start_idx:outage_start_idx+100]:     
    heading_out.append(r['yaw'])
    if 'ground_truth' in r and 'heading' in r['ground_truth']:
       heading_gt_out.append(r['ground_truth']['heading'])

if len(heading_gt_out) == 0:
    for r in traj[outage_start_idx:outage_start_idx+600]:
        heading_gt_out.append(r['gps_speed_heading'])
        
print(f"Yaw standard deviation in pre-outage: {np.std(heading_pre)}")
if len(heading_gt_out) > 0:
    drift = np.unwrap(heading_out)[-1] - np.unwrap(heading_out)[0]
    gt_drift = np.unwrap(heading_gt_out)[min(99, len(heading_gt_out)-1)] - np.unwrap(heading_gt_out)[0]
    print(f"Heading drift during first 10s of outage: {np.degrees(drift):.2f} (Filter) vs GT: {np.degrees(gt_drift):.2f}")

# 3. Check speed bias
speeds_pred = []
speeds_gps = []
for r in traj[outage_start_idx-200:outage_start_idx]:     
    speeds_pred.append(r['speed_fwd'])
    if r['gps_fix_valid']:
       speeds_gps.append(r['gps_speed_mps'])
       
print(f"Pre-outage Mean predicted speed: {np.mean(speeds_pred):.2f}, Mean GPS speed: {np.mean(speeds_gps):.2f}")
if np.mean(speeds_gps) > 0:
    print(f"Ratio pred/gps = {np.mean(speeds_pred)/np.mean(speeds_gps):.3f}")
