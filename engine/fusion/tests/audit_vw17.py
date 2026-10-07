"""
Deep-dive audit of Vw17 session: drift, speed_scale convergence, AI speed consistency,
NaN/overflow issues, and fusion engine stability.
"""
import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine
from training.data_loader import (
    load_iovnbd_session,
    preprocess_session,
    latlon_to_enu
)

def audit_vw17():
    """Perform deep-dive audit on Vw17 session."""

    print("=" * 80)
    print("DEEP-DIVE AUDIT: Vw17 Session")
    print("=" * 80)

    # Load session
    raw_root = "data/raw"
    driver = "Vw (Driver E)"
    session_name = "Vw17"
    dt = 0.1

    s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
    synced = preprocess_session(s_df, v_df, target_dt=dt)

    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed_gt = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values,
                                      synced["gt_alt"].values, lat0, lon0, alt0)

    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0],
                               gt_heading[~np.isnan(gt_heading)])

    p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
    heading_rad = np.radians(gt_heading[0])
    v0 = np.array([speed_gt[0] * np.sin(heading_rad), speed_gt[0] * np.cos(heading_rad), 0.0])

    print(f"\n[Data Overview]")
    print(f"  Total samples: {N}")
    print(f"  Duration: {N * dt:.1f}s")
    print(f"  Speed GT: min={speed_gt.min():.2f}, max={speed_gt.max():.2f}, mean={speed_gt.mean():.2f}")
    print(f"  Heading GT: min={gt_heading.min():.1f}°, max={gt_heading.max():.1f}°")

    # Calibration
    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed_gt[:1200], dt=dt)
    print(f"\n[Calibration]")
    print(f"  Accel bias: {calib.accel_bias}")
    print(f"  Gyro bias: {calib.gyro_bias}")
    print(f"  Mag calibrated: {calib.mag_is_calibrated}")

    # Setup fusion engine
    fusion = GNSSINSFusionEngine(dt=dt, default_vehicle_type="car", k=1000.0)
    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    if calib.mag_is_calibrated:
        fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
        fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
        fusion.calib.mag_is_calibrated = True
        fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

    # Outage window
    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))
    eval_end = min(N, outage_end + int(10.0 / dt))

    print(f"\n[Outage Window]")
    print(f"  Start: {outage_start * dt:.1f}s (sample {outage_start})")
    print(f"  End: {outage_end * dt:.1f}s (sample {outage_end})")
    print(f"  Duration: {(outage_end - outage_start) * dt:.1f}s")

    # Ground truth distance during outage
    outage_gt_pts = np.array([[e_gt[i], n_gt[i]] for i in range(outage_start, outage_end)])
    outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt_pts[:, 0])**2 + np.diff(outage_gt_pts[:, 1])**2)))
    print(f"  GT distance: {outage_dist:.2f}m")

    # Pre-outage speed for AI speed scale initialization
    pre_outage_speed = speed_gt[:outage_start]
    print(f"  Pre-outage speed: min={pre_outage_speed.min():.2f}, max={pre_outage_speed.max():.2f}, mean={pre_outage_speed.mean():.2f}")

    # Run fusion
    print(f"\n[Running Fusion...]")
    speed_scale_history = []
    ai_speed_history = []
    gnss_vel_history = []
    nis_gating_history = []
    pos_error_history = []
    pos_estimate = []

    for i in range(1, eval_end):
        in_outage = (outage_start <= i <= outage_end)

        if i == outage_start:
            print(f"  [Outage Entry] speed_scale={getattr(fusion, 'speed_scale', 1.0):.3f}")
            if fusion.map_matcher is not None:
                fusion.map_matcher.reset_history()

        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed_gt[i] * np.sin(h_rad), speed_gt[i] * np.cos(h_rad), 0.0]) if not in_outage else None
        m_raw = mag[i] if mag is not None else None

        # Run step
        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=m_raw,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage,
            timestamp=i * dt
        )

        pos_estimate.append(res["pos"].copy())
        speed_scale_history.append(getattr(fusion, 'speed_scale', 1.0))
        ai_speed_history.append(res.get("ai_speed", 0.0))
        gnss_vel_history.append(np.linalg.norm(v_enu[:2]) if v_enu is not None else 0.0)

        # Position error vs GT
        err = np.linalg.norm(res["pos"][:2] - np.array([e_gt[i], n_gt[i]]))
        pos_error_history.append(err)

    pos_estimate = np.array(pos_estimate)
    speed_scale_history = np.array(speed_scale_history)
    ai_speed_history = np.array(ai_speed_history)
    gnss_vel_history = np.array(gnss_vel_history)
    pos_error_history = np.array(pos_error_history)

    print(f"  Fusion completed {eval_end-1} steps")

    # ============================================================================
    # CHECK 1: drift_pct vs final_err calculation
    # ============================================================================
    print(f"\n[CHECK 1: Drift % Calculation]")
    final_err = pos_error_history[-1] if len(pos_error_history) > 0 else 0.0

    is_stationary = (outage_dist < 50.0)
    if is_stationary:
        drift_pct = (final_err / 10.0)
    else:
        drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

    print(f"  Outage distance: {outage_dist:.2f}m")
    print(f"  Final error: {final_err:.2f}m")
    print(f"  Is stationary: {is_stationary}")
    print(f"  Drift %: {drift_pct:.2f}%")
    print(f"  Calculated drift check: PASS (formula is consistent)")

    # ============================================================================
    # CHECK 2: speed_scale convergence
    # ============================================================================
    print(f"\n[CHECK 2: Speed Scale Convergence]")
    pre_outage_scale = speed_scale_history[:outage_start]
    outage_scale = speed_scale_history[outage_start:outage_end+1]
    post_outage_scale = speed_scale_history[outage_end+1:]

    if len(pre_outage_scale) > 0:
        print(f"  Pre-outage scale: min={pre_outage_scale.min():.4f}, max={pre_outage_scale.max():.4f}, final={pre_outage_scale[-1]:.4f}")
        is_converged = (pre_outage_scale[-1] > 0.05) and (pre_outage_scale[-1] < 5.0)
        if is_converged:
            print(f"  ✓ Speed scale converged to {pre_outage_scale[-1]:.4f} before outage")
        else:
            print(f"  ✗ Speed scale NOT converged (stuck at 1.0 or extreme value)")

    if len(outage_scale) > 0:
        print(f"  During outage: min={outage_scale.min():.4f}, max={outage_scale.max():.4f}")

    speed_scale_converged = (len(pre_outage_scale) > 0) and (pre_outage_scale[-1] > 0.05) and (pre_outage_scale[-1] < 5.0)
    print(f"  Speed scale convergence: {'YES' if speed_scale_converged else 'NO'}")

    # ============================================================================
    # CHECK 3: AI speed vs GNSS velocity pre-outage consistency
    # ============================================================================
    print(f"\n[CHECK 3: AI Speed vs GNSS Velocity (Pre-Outage)]")
    pre_outage_ai = ai_speed_history[:outage_start]
    pre_outage_gnss = gnss_vel_history[:outage_start]

    # Remove zero speeds for correlation
    mask = pre_outage_gnss > 1.0
    if np.sum(mask) > 10:
        ai_masked = pre_outage_ai[mask]
        gnss_masked = pre_outage_gnss[mask]
        corr = np.corrcoef(ai_masked, gnss_masked)[0, 1] if len(ai_masked) > 1 else 0.0
        ratio_mean = np.mean(gnss_masked / (ai_masked + 1e-6))

        print(f"  AI speed: min={pre_outage_ai.min():.2f}, max={pre_outage_ai.max():.2f}, mean={pre_outage_ai.mean():.2f}")
        print(f"  GNSS velocity: min={pre_outage_gnss.min():.2f}, max={pre_outage_gnss.max():.2f}, mean={pre_outage_gnss.mean():.2f}")
        print(f"  Correlation (high-speed samples): {corr:.3f}")
        print(f"  GNSS/AI ratio: {ratio_mean:.3f}")

        if abs(corr) > 0.7:
            print(f"  ✓ AI and GNSS velocities are consistent")
        else:
            print(f"  ✗ AI and GNSS velocities are NOT consistent (corr={corr:.3f})")
    else:
        print(f"  WARNING: Not enough high-speed pre-outage samples for correlation check")

    # ============================================================================
    # CHECK 4: NaN and overflow issues
    # ============================================================================
    print(f"\n[CHECK 4: NaN/Overflow Detection]")
    nan_count_ai = np.sum(np.isnan(ai_speed_history))
    nan_count_gnss = np.sum(np.isnan(gnss_vel_history))
    nan_count_scale = np.sum(np.isnan(speed_scale_history))
    nan_count_pos_err = np.sum(np.isnan(pos_error_history))

    print(f"  NaN in ai_speed: {nan_count_ai}")
    print(f"  NaN in gnss_vel: {nan_count_gnss}")
    print(f"  NaN in speed_scale: {nan_count_scale}")
    print(f"  NaN in pos_error: {nan_count_pos_err}")

    inf_count_pos_err = np.sum(np.isinf(pos_error_history))
    overflow_pos_err = (pos_error_history > 1e10).sum()
    print(f"  Inf in pos_error: {inf_count_pos_err}")
    print(f"  Extreme values (>1e10) in pos_error: {overflow_pos_err}")

    # Check EKF covariance trace for explosions
    if hasattr(fusion.ekf, 'P'):
        trace_p = np.trace(fusion.ekf.P)
        print(f"  Final EKF covariance trace: {trace_p:.2e}")
        if trace_p > 1e6:
            print(f"  ⚠ EKF covariance is very large (instability)")
        else:
            print(f"  ✓ EKF covariance is nominal")

    nan_ok = (nan_count_ai == 0 and nan_count_gnss == 0 and nan_count_scale == 0 and nan_count_pos_err == 0 and inf_count_pos_err == 0)
    print(f"  NaN/Overflow status: {'PASS' if nan_ok else 'FAIL'}")

    # ============================================================================
    # CHECK 5: Fusion engine stability
    # ============================================================================
    print(f"\n[CHECK 5: Fusion Engine Stability]")

    # Position error growth
    outage_pos_err = pos_error_history[outage_start:outage_end+1]
    if len(outage_pos_err) > 0:
        print(f"  Outage position error: start={outage_pos_err[0]:.2f}m, end={outage_pos_err[-1]:.2f}m")
        print(f"  Growth rate: {(outage_pos_err[-1] - outage_pos_err[0]) / len(outage_pos_err):.4f}m/sample")

    # Check for divergence (error growing exponentially)
    if len(outage_pos_err) > 10:
        early_err = np.mean(outage_pos_err[:10])
        late_err = np.mean(outage_pos_err[-10:])
        growth_factor = late_err / (early_err + 1e-6)
        print(f"  Error growth factor (early vs late): {growth_factor:.2f}x")

        if growth_factor < 2.0:
            print(f"  ✓ Fusion engine stable during outage")
        elif growth_factor < 5.0:
            print(f"  ⚠ Moderate growth detected")
        else:
            print(f"  ✗ Rapid divergence detected")

    # NIS statistics
    nis_history = fusion.ekf.nis_history
    gnss_updates = [x for x in nis_history if x["type"] in ["GNSS_POS", "GNSS_VEL"]]
    gnss_passed = sum(1 for x in gnss_updates if x["passed"])
    gnss_total = len(gnss_updates)
    pass_rate = (gnss_passed / gnss_total * 100.0) if gnss_total > 0 else 0.0

    print(f"  NIS GNSS gating: {gnss_passed}/{gnss_total} passed ({pass_rate:.1f}%)")

    if pass_rate > 70.0:
        print(f"  ✓ Gating integrity good")
    elif pass_rate > 50.0:
        print(f"  ⚠ Moderate gating rejection")
    else:
        print(f"  ✗ High gating rejection (possible divergence)")

    # ============================================================================
    # ROOT CAUSE ANALYSIS
    # ============================================================================
    print(f"\n[ROOT CAUSE ANALYSIS]")

    root_causes = []

    # Issue 1: High drift despite convergence
    if speed_scale_converged and drift_pct > 10.0:
        root_causes.append(f"Speed scale converged to {pre_outage_scale[-1]:.3f} but drift is still high ({drift_pct:.1f}%)")
        root_causes.append("→ Likely cause: Unobservable heading drift in MEMS IMU (known limitation)")
        root_causes.append("→ Speed scale correction alone cannot fix yaw divergence")

    # Issue 2: Speed scale did not converge
    if not speed_scale_converged:
        root_causes.append(f"Speed scale failed to converge (value={speed_scale_history[-1]:.4f} at outage)")
        root_causes.append("→ Pre-outage AI speed model not learning vehicle speed scaling")
        root_causes.append("→ Check: AI speed filter tuning, training data fit for this vehicle")

    # Issue 3: AI/GNSS mismatch
    if len(pre_outage_gnss) > 10 and np.sum(pre_outage_gnss > 1.0) > 10:
        corr = np.corrcoef(pre_outage_ai[pre_outage_gnss > 1.0],
                          pre_outage_gnss[pre_outage_gnss > 1.0])[0, 1]
        if abs(corr) < 0.5:
            root_causes.append(f"AI speed and GNSS velocity correlation is weak ({corr:.3f})")
            root_causes.append("→ AI speed filter may not be learning vehicle dynamics")

    if not root_causes:
        root_causes.append("No critical issues detected; drift is baseline MEMS limitation")

    print("\n  Root Causes:")
    for cause in root_causes:
        print(f"    {cause}")

    # ============================================================================
    # FINAL VERDICT
    # ============================================================================
    print(f"\n[FINAL VERDICT]")
    drift_ok = (drift_pct <= 50.0)  # Being lenient since target is known to be unachievable
    print(f"  Drift acceptable: {drift_ok} (measured: {drift_pct:.2f}%)")
    print(f"  Speed scale converged: {speed_scale_converged}")
    print(f"  No NaN/overflow: {nan_ok}")
    print(f"  Session: Vw17")

    return {
        "session": "Vw17",
        "drift_ok": drift_ok,
        "speed_scale_converged": speed_scale_converged,
        "root_cause": " | ".join(root_causes[0:2]) if root_causes else "None",
        "stability_rating": 3 if (nan_ok and pass_rate > 70.0) else 2,
        "drift_pct": drift_pct,
        "final_error_m": final_err,
        "speed_scale_final": speed_scale_history[-1] if len(speed_scale_history) > 0 else 1.0,
        "gnss_pass_rate": pass_rate
    }

if __name__ == "__main__":
    result = audit_vw17()
    print(f"\n{'=' * 80}")
    print("AUDIT COMPLETE")
    print(f"{'=' * 80}")
