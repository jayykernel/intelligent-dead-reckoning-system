"""
Deep-dive audit of Vw16a session with root cause analysis.
Returns structured output as required by workflow task.
"""
import os
import sys
import numpy as np
import json

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from eval.run_full_benchmark import evaluate_dead_reckoning_session, ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

def audit_vw16a():
    raw_root = "data/raw"
    try:
        s_df, v_df = load_iovnbd_session(raw_root, "Vw (Driver E)", "Vw16a")
    except Exception as e:
        return {
            "session": "Vw16a",
            "drift_ok": False,
            "speed_scale_converged": False,
            "root_cause": f"Data load failed: {str(e)}"
        }

    synced = preprocess_session(s_df, v_df, target_dt=0.1)
    N = len(synced)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

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

    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / 0.1)
    outage_end = min(outage_end, N - int(10.0 / 0.1))
    eval_end = min(N, outage_end + int(10.0 / 0.1))

    fusion = ProductionMobileFusionEngine(dt=0.1, default_vehicle_type="car", k=1000.0)
    rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
    fusion.road_network = rn
    from engine.map_matching.hmm_matcher import HMMMapMatcher
    fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")

    if calib.mag_is_calibrated:
        fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
        fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
        fusion.calib.mag_is_calibrated = True
        fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    # Track key metrics
    speed_scales = []
    ai_speeds_pre = []
    gnss_speeds_pre = []
    has_nans = False
    overflow_issues = []

    # Pre-outage analysis
    print("=== PRE-OUTAGE ANALYSIS ===")
    for i in range(1, min(outage_start, 100)):
        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])

        acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])
        ai_s, sigma_ai, q_scale = fusion.ai_corrector.process_imu_sample(acc[i], gyro[i])
        gnss_s = np.linalg.norm(v_enu[:2])

        # Check for overflow in EKF covariance
        if np.any(np.abs(fusion.ekf.P) > 1e6):
            overflow_issues.append(f"Step {i}: Large covariance values")

        if ai_s is not None and gnss_s > 1.0:
            ai_speeds_pre.append(ai_s)
            gnss_speeds_pre.append(gnss_s)

        fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=mag[i] if mag is not None else None,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=True,
            timestamp=i * 0.1
        )

        speed_scales.append(fusion.speed_scale)

    print(f"Initial speed_scale: {speed_scales[-1]:.4f}")

    # Now run full session to get final error
    print("\n=== FULL OUTAGE SIMULATION ===")
    fusion._speed_scale_updates = 0  # Reset for clean analysis
    fusion.speed_scale = 1.0

    for i in range(1, eval_end):
        in_outage = (outage_start <= i <= outage_end)

        if i == outage_start:
            print(f'[Vw16a] Entering outage at step {i}, speed_scale={fusion.speed_scale:.3f}')

        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

        fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=mag[i] if mag is not None else None,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage,
            timestamp=i * 0.1
        )

        # Check for NaN/overflow
        if np.any(np.isnan(fusion.ekf.P)):
            has_nans = True
            print(f"NaN detected at step {i}")

        if np.any(np.abs(fusion.ekf.P) > 1e10):
            overflow_issues.append(f"Step {i}: Extreme covariance values >1e10")

    # Calculate error
    outage_gt_pts = []
    for i in range(outage_start, outage_end + 1):
        outage_gt_pts.append([e_gt[i], n_gt[i]])

    outage_gt_pts = np.array(outage_gt_pts)
    if len(outage_gt_pts) > 1:
        outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt_pts[:, 0])**2 + np.diff(outage_gt_pts[:, 1])**2)))
    else:
        outage_dist = 0.0

    final_err = np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[outage_end], n_gt[outage_end]]))

    is_stationary = (outage_dist < 50.0)
    if is_stationary:
        drift_pct = (final_err / 10.0)
    else:
        drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

    # Check drift calculation accuracy
    drift_calc_check = (final_err / outage_dist * 100.0) if outage_dist > 0 else 0.0

    print(f"\n=== FINAL RESULTS ===")
    print(f"Outage Distance: {outage_dist:.2f} m")
    print(f"Final Error: {final_err:.2f} m")
    print(f"Drift %: {drift_pct:.2f} %")
    print(f"Speed Scale Final: {fusion.speed_scale:.4f}")
    print(f"Speed Scale Converged: {abs(fusion.speed_scale - 1.0) < 0.2}")

    # Root cause analysis
    root_causes = []

    # 1. Speed scale learning check
    if abs(fusion.speed_scale - 1.0) < 0.01:
        root_causes.append("speed_scale stuck near 1.0 (not learning)")
    elif fusion.speed_scale < 0.8 or fusion.speed_scale > 1.5:
        root_causes.append(f"speed_scale extreme ({fusion.speed_scale:.2f})")

    # 2. AI vs GNSS consistency
    if ai_speeds_pre and gnss_speeds_pre:
        ai_mean = np.mean(ai_speeds_pre)
        gnss_mean = np.mean(gnss_speeds_pre)
        ratio = ai_mean / gnss_mean if gnss_mean > 0 else 1.0
        if ratio < 0.7 or ratio > 1.3:
            root_causes.append(f"AI-GNSS speed mismatch (AI={ai_mean:.1f}, GNSS={gnss_mean:.1f}, ratio={ratio:.2f})")

    # 3. NaN/Overflow
    if has_nans:
        root_causes.append("NaN in EKF state/covariance")
    if overflow_issues:
        root_causes.append("covariance overflow detected")

    # 4. Drift analysis
    if drift_pct > 10.0:
        root_causes.append(f"high drift {drift_pct:.1f}% > 10% target")
    if abs(drift_calc_check - drift_pct) > 0.5:
        root_causes.append("drift calculation discrepancy")

    # 5. Fusion stability
    nis_history = fusion.ekf.nis_history
    if nis_history:
        failed_updates = sum(1 for x in nis_history if not x.get("passed", True))
        if failed_updates > len(nis_history) * 0.1:
            root_causes.append(f"high NIS failure rate ({failed_updates}/{len(nis_history)})")

    root_cause = "; ".join(root_causes) if root_causes else "No major issues detected"

    # Check if drift is OK (within 10% target)
    drift_ok = drift_pct <= 10.0

    # Check if speed scale converged
    speed_scale_converged = abs(fusion.speed_scale - 1.0) < 0.2

    return {
        "session": "Vw16a",
        "drift_ok": drift_ok,
        "speed_scale_converged": speed_scale_converged,
        "root_cause": root_cause
    }

if __name__ == "__main__":
    result = audit_vw16a()
    print("\n=== WORKFLOW OUTPUT ===")
    print(json.dumps(result, indent=2))
    print("\n=== SUMMARY ===")
    print(f"Session: {result['session']}")
    print(f"Drift OK: {result['drift_ok']}")
    print(f"Speed Scale Converged: {result['speed_scale_converged']}")
    print(f"Root Cause: {result['root_cause']}")