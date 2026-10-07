import sys, os, glob
from eval.run_full_benchmark import ProductionMobileFusionEngine
from training.data_loader import preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
import numpy as np
import pandas as pd

dt = 0.1
s_files = glob.glob("data/raw/**/Vta30/S-*.csv", recursive=True)
v_files = glob.glob("data/raw/**/Vta30/[Vv]-*.csv", recursive=True)
s_df = pd.read_csv(s_files[0], encoding='latin1')
v_df = pd.read_csv(v_files[0], encoding='latin1')
s_df.columns = [c.strip() for c in s_df.columns]
v_df.columns = [c.strip() for c in v_df.columns]
synced = preprocess_session(s_df, v_df, target_dt=dt)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=100.0)

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

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

outage_start = 3000
outage_end = 3600

# Audit results
audit = {
    "drift_pct_calculation": None,
    "speed_scale_convergence": None,
    "ai_speed_consistency": None,
    "nan_or_overflow": False,
    "stability_issues": []
}

# Track metrics during pre-outage
pre_outage_scales = []
pre_outage_ai_speeds = []
pre_outage_gnss_speeds = []

# Track metrics during outage
outage_ai_speeds = []
outage_positions = []

for i in range(1, len(synced)):
    in_outage = (outage_start <= i < outage_end)
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

    res = fusion.step(
        acc_raw=acc[i], gyro_raw=gyro[i],
        gnss_pos_enu=pos_enu, gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage, timestamp=i*dt,
        gnss_acc_m=2.0 if not in_outage else None,
    )
    
    # Check for NaNs
    if np.any(np.isnan(res['pos'])) or np.any(np.isnan(res['vel'])) or np.any(np.isnan(res['cov_2d'])):
        audit["nan_or_overflow"] = True
        audit["stability_issues"].append(f"NaN at step {i}")
    
    # Collect pre-outage data
    if not in_outage and i < outage_start:
        scale = getattr(fusion, 'speed_scale', 1.0)
        pre_outage_scales.append(scale)
        if res['ai_speed'] is not None and res['ai_speed'] > 0:
            pre_outage_ai_speeds.append(res['ai_speed'])
        gnss_speed_2d = np.linalg.norm(v_enu[:2]) if v_enu is not None else 0.0
        if gnss_speed_2d > 0.5:
            pre_outage_gnss_speeds.append(gnss_speed_2d)
    
    # Collect outage data
    if in_outage:
        if res['ai_speed'] is not None and res['ai_speed'] > 0:
            outage_ai_speeds.append(res['ai_speed'])
        outage_positions.append(res['pos'])

# Check 1: speed_scale convergence
if pre_outage_scales:
    final_scale = pre_outage_scales[-1]
    median_scale = np.median(pre_outage_scales[-100:]) if len(pre_outage_scales) >= 100 else np.median(pre_outage_scales)
    # Consider converged if within 5% of median in last 100 updates or if scale != 1.0
    if abs(final_scale - median_scale) < 0.05 * median_scale and final_scale != 1.0:
        audit["speed_scale_convergence"] = True
    else:
        audit["speed_scale_convergence"] = False
    print(f"Speed scale: final={final_scale:.3f}, median_last={median_scale:.3f}")

# Check 2: AI speed vs GNSS speed consistency pre-outage
if pre_outage_ai_speeds and pre_outage_gnss_speeds:
    ai_mean = np.mean(pre_outage_ai_speeds)
    gnss_mean = np.mean(pre_outage_gnss_speeds)
    ratio = ai_mean / gnss_mean if gnss_mean > 0 else 1.0
    # Consistent if within 30% of each other
    if 0.7 <= ratio <= 1.3:
        audit["ai_speed_consistency"] = True
    else:
        audit["ai_speed_consistency"] = False
    print(f"AI speed consistency: AI mean={ai_mean:.2f}, GNSS mean={gnss_mean:.2f}, ratio={ratio:.3f}")

# Check 3: Drift % calculation vs final error
if outage_positions:
    outage_dist = np.linalg.norm(e_gt[outage_end-1:outage_end] - e_gt[outage_start:outage_start+1])
    final_pos = outage_positions[-1]
    final_err = np.linalg.norm(final_pos[:2] - np.array([e_gt[outage_end-1], n_gt[outage_end-1]]))
    if outage_dist > 0:
        drift_pct = (final_err / outage_dist) * 100.0
    else:
        drift_pct = final_err / 10.0
    
    print(f"Drift calc: outage_dist={outage_dist:.2f}m, final_err={final_err:.2f}m, drift_pct={drift_pct:.2f}%")
    
    # Mark as OK if calculation is consistent (not NaN, not Inf)
    if np.isfinite(drift_pct):
        audit["drift_pct_calculation"] = True
    else:
        audit["drift_pct_calculation"] = False

print("\n=== AUDIT SUMMARY ===")
print(f"drift_pct_calculation OK: {audit['drift_pct_calculation']}")
print(f"speed_scale_converged: {audit['speed_scale_convergence']}")
print(f"ai_speed_consistency: {audit['ai_speed_consistency']}")
print(f"nan_or_overflow: {audit['nan_or_overflow']}")
if audit["stability_issues"]:
    print(f"Stability issues: {audit['stability_issues']}")

