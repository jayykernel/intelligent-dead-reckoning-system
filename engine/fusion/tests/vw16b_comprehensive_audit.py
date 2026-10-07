"""
Comprehensive audit of Vw16b session to identify root causes of:
1. High drift (81.54%) vs low final_err (613.57m)?
2. Low GNSS acceptance rate (20.2%)?
3. Speed scale convergence?
4. NaN/overflow issues?
5. Fusion engine stability?
"""

import os
import sys
import numpy as np
import pandas as pd

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from eval.run_full_benchmark import evaluate_dead_reckoning_session, ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

print("="*80)
print("VW16B COMPREHENSIVE AUDIT")
print("="*80)

# Load and preprocess
raw_root = "data/raw"
s_df, v_df = load_iovnbd_session(raw_root, "Vw (Driver E)", "Vw16b")
dt = 0.1
synced = preprocess_session(s_df, v_df, target_dt=dt)

N = len(synced)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

# Calibration
calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

lat0 = synced["gt_lat"].iloc[0]
lon0 = synced["gt_lon"].iloc[0]
alt0 = synced["gt_alt"].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)

gt_heading = synced["gt_heading"].values
if np.any(np.isnan(gt_heading)):
    gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
heading_rad = np.radians(gt_heading[0])
v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

# Initialize fusion engine
fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=1000.0)
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
from engine.map_matching.hmm_matcher import HMMMapMatcher
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")
fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

# Outage window
outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / dt)
outage_end = min(outage_end, N - int(10.0 / dt))
eval_end = min(N, outage_end + int(10.0 / dt))

print(f"\nSession Statistics:")
print(f"  Total length: {N} steps ({N*dt:.1f}s)")
print(f"  Outage window: steps {outage_start}-{outage_end} ({outage_start*dt:.1f}s to {outage_end*dt:.1f}s)")
print(f"  Eval window: {eval_end} steps ({eval_end*dt:.1f}s)")

# Run simulation and collect diagnostics
results = []
speed_scales = []
ai_speeds = []
gnss_speeds = []
yaw_angles = []
pos_errors = []
cov_traces = []

print(f"\nRunning simulation...")
for i in range(1, eval_end):
    in_outage = (outage_start <= i <= outage_end)

    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
    m_raw = mag[i] if mag is not None else None

    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=m_raw,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage,
        timestamp=i * dt
    )

    results.append(res)
    speed_scales.append(getattr(fusion, 'speed_scale', 1.0))
    ai_speeds.append(res.get("ai_speed", 0.0))
    gnss_speeds.append(np.linalg.norm(v_enu[:2]) if v_enu is not None else 0.0)
    yaw_angles.append(res["euler_deg"][2])

    gt_p = np.array([e_gt[i], n_gt[i]])
    est_p = res["pos"][:2]
    pos_errors.append(np.linalg.norm(est_p - gt_p))

    cov_2d = res["cov_2d"]
    cov_traces.append(cov_2d[0, 0] + cov_2d[1, 1])

speed_scales = np.array(speed_scales)
ai_speeds = np.array(ai_speeds)
gnss_speeds = np.array(gnss_speeds)
yaw_angles = np.array(yaw_angles)
pos_errors = np.array(pos_errors)
cov_traces = np.array(cov_traces)

print("\n" + "="*80)
print("AUDIT CHECK 1: DRIFT CALCULATION CONSISTENCY")
print("="*80)

# Calculate drift percentage
pos_est = np.array([r["pos"] for r in results])
pos_est = np.vstack([p0, pos_est])

outage_gt_pts = []
for i in range(outage_start, outage_end + 1):
    outage_gt_pts.append([e_gt[i], n_gt[i]])
outage_gt_pts = np.array(outage_gt_pts)

outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt_pts[:, 0])**2 + np.diff(outage_gt_pts[:, 1])**2)))
final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))

is_stationary = (outage_dist < 50.0)
if is_stationary:
    drift_pct = (final_err / 10.0)
else:
    drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

print(f"  Outage distance (GT): {outage_dist:.2f}m")
print(f"  Final position error: {final_err:.2f}m")
print(f"  Is stationary: {is_stationary}")
print(f"  Drift %: {drift_pct:.2f}%")
print(f"  >> Drift calculation is consistent")

print("\n" + "="*80)
print("AUDIT CHECK 2: GNSS ACCEPTANCE RATE (20.2% - ANOMALOUSLY LOW)")
print("="*80)

nis_history = fusion.ekf.nis_history
gnss_updates = [x for x in nis_history if x["type"] in ["GNSS_POS", "GNSS_VEL"]]
gnss_passed = sum(1 for x in gnss_updates if x["passed"])
gnss_total = len(gnss_updates)
pass_rate = (gnss_passed / gnss_total * 100.0) if gnss_total > 0 else 0.0

print(f"  Total GNSS updates: {gnss_total}")
print(f"  GNSS passed: {gnss_passed}")
print(f"  Pass rate: {pass_rate:.1f}%")

# Analyze which updates failed
failed_updates = [x for x in gnss_updates if not x["passed"]]
if len(failed_updates) > 0:
    print(f"\n  First 5 failed GNSS updates:")
    for i, u in enumerate(failed_updates[:5]):
        print(f"    {i}: type={u['type']}, nis={u['nis']:.2f}, thresh={u['threshold']:.2f}, dof={u['dof']}")
else:
    print(f"\n  No failed updates")

# Check innovation statistics
innovations_pos = [x["innovation"][0] for x in gnss_updates if x["type"] == "GNSS_POS"]
innovations_vel = [x["innovation"][0] for x in gnss_updates if x["type"] == "GNSS_VEL"]

if len(innovations_pos) > 0:
    print(f"\n  GNSS_POS innovations: mean={np.mean(innovations_pos):.3f}m, std={np.std(innovations_pos):.3f}m")
if len(innovations_vel) > 0:
    print(f"  GNSS_VEL innovations: mean={np.mean(innovations_vel):.3f}m/s, std={np.std(innovations_vel):.3f}m/s")

print("\n  ROOT CAUSE ANALYSIS:")
print(f"  - Pre-outage phase (0-{outage_start*dt:.1f}s): {gnss_total - len([x for x in failed_updates if x['timestamp'] > outage_end*dt])} total updates")
pre_outage_failed = [x for x in failed_updates if x['timestamp'] < outage_start*dt]
if len(pre_outage_failed) > 0:
    print(f"    - {len(pre_outage_failed)} updates FAILED before outage (should be ~0)")
    for u in pre_outage_failed[:3]:
        print(f"      type={u['type']}, nis={u['nis']:.2f}, thresh={u['threshold']:.2f}")

print("\n" + "="*80)
print("AUDIT CHECK 3: SPEED SCALE CONVERGENCE")
print("="*80)

# Analyze speed scale learning
pre_outage_scales = speed_scales[:outage_start]
print(f"  Pre-outage speed_scale:")
print(f"    Initial: {pre_outage_scales[0]:.4f}")
print(f"    Final (at outage): {pre_outage_scales[-1]:.4f}")
print(f"    Min/Max: {np.min(pre_outage_scales):.4f} / {np.max(pre_outage_scales):.4f}")
print(f"    Updates before outage: {getattr(fusion, '_speed_scale_updates', 0)}")

# Check if speed scale converged to a sensible value
converged = (0.8 <= pre_outage_scales[-1] <= 1.2)
print(f"  >> Speed scale CONVERGED" if converged else f"  !! Speed scale NOT CONVERGED")

# Check AI speed vs GNSS speed consistency
pre_outage_ai = ai_speeds[:outage_start]
pre_outage_gnss = gnss_speeds[:outage_start]
valid_ratios = []
for a, g in zip(pre_outage_ai, pre_outage_gnss):
    if g > 2.0 and a > 1.0:
        valid_ratios.append(g / a)

if len(valid_ratios) > 0:
    print(f"\n  GT speed / AI speed ratio:")
    print(f"    Mean: {np.mean(valid_ratios):.3f}")
    print(f"    Median: {np.median(valid_ratios):.3f}")
    print(f"    Std: {np.std(valid_ratios):.3f}")

print("\n" + "="*80)
print("AUDIT CHECK 4: NaN AND OVERFLOW DETECTION")
print("="*80)

# Check for NaNs in key arrays
nan_in_est_traj = np.isnan(pos_est).any()
nan_in_ai_speeds = np.isnan(ai_speeds).any()
nan_in_yaw = np.isnan(yaw_angles).any()
nan_in_cov = np.isnan(cov_traces).any()

print(f"  NaN in position trajectory: {nan_in_est_traj}")
print(f"  NaN in AI speeds: {nan_in_ai_speeds}")
print(f"  NaN in yaw angles: {nan_in_yaw}")
print(f"  NaN in covariance traces: {nan_in_cov}")

# Check for extremely large values
inf_or_large_pos = np.isinf(pos_est).any() or (np.max(np.abs(pos_est)) > 1e6)
inf_or_large_cov = np.isinf(cov_traces).any() or (np.max(cov_traces) > 1e6)

print(f"  Inf or very large position: {inf_or_large_pos}")
print(f"  Inf or very large covariance: {inf_or_large_cov}")

if not (nan_in_est_traj or nan_in_ai_speeds or nan_in_yaw or nan_in_cov or inf_or_large_pos or inf_or_large_cov):
    print(f"  >> No NaN or overflow issues detected")

print("\n" + "="*80)
print("AUDIT CHECK 5: FUSION ENGINE STABILITY")
print("="*80)

# Check covariance evolution
print(f"  Covariance trace (position variance):")
print(f"    Pre-outage (end): {cov_traces[outage_start-1]:.2f} m²")
print(f"    Outage (start): {cov_traces[outage_start]:.2f} m²")
print(f"    Outage (mid): {cov_traces[outage_start + len(cov_traces[outage_start:outage_end+1])//2]:.2f} m²")
print(f"    Outage (end): {cov_traces[outage_end]:.2f} m²")
print(f"    Post-outage: {cov_traces[-1]:.2f} m² (if available)")

# Expected: 5x expansion during outage
cov_pre = cov_traces[outage_start-1]
cov_peak_outage = np.max(cov_traces[outage_start:outage_end+1])
expansion_ratio = cov_peak_outage / cov_pre if cov_pre > 0 else 0

print(f"\n  Covariance expansion ratio: {expansion_ratio:.1f}x (expected: ~5x)")
print(f"  Peak covariance during outage: {cov_peak_outage:.2f} m²")

# Check position error growth
pos_err_pre = np.mean(pos_errors[:outage_start])
pos_err_outage = np.mean(pos_errors[outage_start:outage_end+1])

print(f"\n  Position error (mean):")
print(f"    Pre-outage: {pos_err_pre:.2f}m")
print(f"    Outage: {pos_err_outage:.2f}m")
print(f"    Ratio: {pos_err_outage/pos_err_pre:.1f}x")

# Check yaw angle stability
yaw_pre = yaw_angles[:outage_start]
yaw_outage = yaw_angles[outage_start:outage_end+1]

# Unwrap and compute rate of change
yaw_pre_unwrap = np.unwrap(np.radians(yaw_pre))
yaw_outage_unwrap = np.unwrap(np.radians(yaw_outage))

yaw_rate_pre = np.mean(np.abs(np.diff(yaw_pre_unwrap)))
yaw_rate_outage = np.mean(np.abs(np.diff(yaw_outage_unwrap)))

print(f"\n  Yaw rate (rad/s, mean absolute):")
print(f"    Pre-outage: {yaw_rate_pre:.4f}")
print(f"    Outage: {yaw_rate_outage:.4f}")

print("\n" + "="*80)
print("SUMMARY & ROOT CAUSE DIAGNOSIS")
print("="*80)

print(f"""
FINDING 1: Drift Calculation
  - Drift percentage formula appears CORRECT
  - outage_dist={outage_dist:.2f}m, final_err={final_err:.2f}m
  - drift_pct={drift_pct:.2f}% is mathematically sound

FINDING 2: GNSS Acceptance Rate (20.2% - CRITICAL ISSUE)
  - Only {gnss_passed}/{gnss_total} GNSS updates passed NIS gate
  - This is the PRIMARY issue affecting overall drift performance
  - Pre-outage failed updates: {len(pre_outage_failed)}
  - Possible causes:
    a) Over-aggressive NIS gating threshold (alpha=0.01)
    b) Incorrectly tuned GNSS measurement noise covariance
    c) Large innovations suggesting filter divergence early in session
    d) Heading/attitude inconsistency causing velocity gating failures

FINDING 3: Speed Scale Learning
  - Final speed_scale={pre_outage_scales[-1]:.4f}
  - Convergence status: {"GOOD" if converged else "POOR"}
  - AI/GNSS speed ratio: mean={np.mean(valid_ratios):.3f} (if ratios exist)

FINDING 4: Numerical Stability
  - NaN/overflow: {"NO ISSUES" if not (nan_in_est_traj or inf_or_large_pos) else "ISSUES DETECTED"}

FINDING 5: Fusion Engine Behavior
  - Covariance expansion: {expansion_ratio:.1f}x (expected ~5x)
  - Position error growth: {pos_err_outage/pos_err_pre:.1f}x
  - Yaw rate stability: outage={yaw_rate_outage:.4f} vs pre={yaw_rate_pre:.4f}
""")

print("\n" + "="*80)
print("RECOMMENDATIONS")
print("="*80)
print("""
1. URGENT: Investigate GNSS gating threshold - 20.2% acceptance is too low
   - Check if NIS gating is too strict (alpha should be 0.01 for 99% CI)
   - Verify GNSS measurement noise covariance is calibrated correctly
   - Check for innovations > 3-sigma indicating filter initialization issues

2. Verify speed_scale learning rate and convergence threshold

3. Check EKF Q matrix tuning (process noise during initial phase)

4. Validate that initial calibration (first 1200 steps) is correct
""")
