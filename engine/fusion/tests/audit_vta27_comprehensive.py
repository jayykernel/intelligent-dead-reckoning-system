"""
Comprehensive audit of Vta27 session for drift root cause analysis.
Checks:
1. drift_pct calculation vs final_err
2. speed_scale convergence
3. AI speed vs GNSS velocity consistency
4. NaN/overflow issues
5. Fusion engine stability
"""
import sys
import numpy as np
sys.path.insert(0, r"C:\dev\dead reckoning proto")

from eval.run_full_benchmark import evaluate_dead_reckoning_session

print("="*80)
print("VTA27 COMPREHENSIVE AUDIT")
print("="*80)

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})

start = res["outage_start"]
end = res["outage_end"]
outage_dist = res["outage_dist_m"]
final_err = res["final_error_m"]
drift_pct = res["drift_pct"]

print(f"\n1. DRIFT_PCT CALCULATION VALIDATION")
print(f"   outage_distance: {outage_dist:.2f} m")
print(f"   final_error: {final_err:.2f} m")
print(f"   drift_pct reported: {drift_pct:.2f}%")
calc_drift = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0
print(f"   drift_pct calculated: {calc_drift:.2f}%")
print(f"   Match: {abs(drift_pct - calc_drift) < 0.01}")

# Detailed outage position error analysis
gt = res["gt_traj"]
est = res["est_traj"]
outage_gt = gt[start:end]
outage_est = est[start:end]
errors = np.linalg.norm(outage_est - outage_gt, axis=1)

print(f"\n2. SPEED_SCALE CONVERGENCE")
results = res["results"]
pre_outage_results = results[max(0, start-100):start]
speed_scales_pre = [r["speed_scale"] for r in pre_outage_results]
outage_results = results[start:end]
speed_scales_outage = [r["speed_scale"] for r in outage_results]

print(f"   Pre-outage speed_scale: min={np.min(speed_scales_pre):.4f}, max={np.max(speed_scales_pre):.4f}, mean={np.mean(speed_scales_pre):.4f}")
print(f"   At outage entry: {speed_scales_pre[-1]:.4f}")
print(f"   During outage: min={np.min(speed_scales_outage):.4f}, max={np.max(speed_scales_outage):.4f}, mean={np.mean(speed_scales_outage):.4f}")
print(f"   Converged (variance < 0.001): {np.var(speed_scales_pre) < 0.001}")

print(f"\n3. AI SPEED vs GNSS VELOCITY CONSISTENCY (PRE-OUTAGE)")
ai_speeds_pre = [r["ai_speed"] for r in pre_outage_results]
# Reconstruct true speeds from GT trajectory
gt_speeds_pre = []
for i in range(1, len(outage_gt[:len(pre_outage_results)])):
    dt = 0.1
    dist = np.linalg.norm(gt[start-100+i] - gt[start-100+i-1])
    gt_speeds_pre.append(dist / dt)

print(f"   AI speed (pre-outage): mean={np.mean(ai_speeds_pre):.2f} m/s, std={np.std(ai_speeds_pre):.2f}")
print(f"   GT speed (pre-outage): mean={np.mean(gt_speeds_pre):.2f} m/s, std={np.std(gt_speeds_pre):.2f}")
print(f"   Bias (AI - GT): {np.mean(ai_speeds_pre) - np.mean(gt_speeds_pre):.2f} m/s")

print(f"\n4. NaN/OVERFLOW CHECKS")
print(f"   Checking EKF states for NaN/Inf during outage...")

has_nan = False
has_inf = False
max_pos_err = 0.0
for i in range(start, end):
    r = results[i]
    pos = r["pos"]
    vel = r["vel"]
    cov = r["cov_2d"]
    euler = r["euler_deg"]

    if np.any(np.isnan(pos)) or np.any(np.isnan(vel)) or np.any(np.isnan(cov)) or np.any(np.isnan(euler)):
        has_nan = True
        print(f"   NaN detected at t={i*0.1:.1f}s")

    if np.any(np.isinf(pos)) or np.any(np.isinf(vel)) or np.any(np.isinf(cov)) or np.any(np.isinf(euler)):
        has_inf = True
        print(f"   Inf detected at t={i*0.1:.1f}s")

    err = errors[i-start]
    max_pos_err = max(max_pos_err, err)

print(f"   Has NaN: {has_nan}")
print(f"   Has Inf: {has_inf}")
print(f"   Max position error: {max_pos_err:.2f} m")

print(f"\n5. FUSION ENGINE STABILITY")
print(f"   Position error growth:")
for i in [0, 150, 300, 450, 600]:
    if start + i < end:
        t = (start + i) * 0.1
        err = errors[i]
        print(f"     t={t:.1f}s: {err:.1f}m")

# Heading stability
print(f"\n   Heading stability (yaw):")
yaws = [r["euler_deg"][2] for r in outage_results]
yaw_diffs = np.abs(np.diff(yaws))
print(f"     Mean abs heading change per step: {np.mean(yaw_diffs):.2f}°")
print(f"     Max heading change per step: {np.max(yaw_diffs):.2f}°")
print(f"     Heading std dev: {np.std(yaws):.2f}°")

# Covariance trace
print(f"\n   Covariance growth (position uncertainty):")
covs = [np.trace(r["cov_2d"]) for r in outage_results]
for i in [0, 150, 300, 450, 600]:
    if i < len(covs):
        t = (start + i) * 0.1
        print(f"     t={t:.1f}s: trace(P_pos)={covs[i]:.2f}")

print(f"\n6. UPDATE ACCEPTANCE (NIS GATING)")
print(f"   GNSS acceptance: {res['gnss_passed']}/{res['gnss_total']} ({res['nis_pass_rate']:.1f}%)")

print(f"\n7. ROOT CAUSE SUMMARY")
print(f"   Drift_pct/final_err consistency: {'OK' if abs(drift_pct - calc_drift) < 0.01 else 'MISMATCH'}")
print(f"   Speed_scale learning: {'CONVERGED' if np.var(speed_scales_pre) < 0.001 else 'NOT CONVERGED'}")
print(f"   AI speed bias: {abs(np.mean(ai_speeds_pre) - np.mean(gt_speeds_pre)):.2f} m/s")
print(f"   Numerical stability: {'STABLE' if not has_nan and not has_inf else 'UNSTABLE'}")
print(f"   Heading divergence: {'SEVERE' if np.std(yaws) > 45 else 'OK'}")
print(f"   Covariance inflation: {covs[-1] / covs[0]:.1f}x growth")

print("\n" + "="*80)
