"""
engine/fusion/tests/debug_vta29.py
Deep diagnostic for session Vta29.
"""
import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.map_matching.hmm_matcher import HMMMapMatcher

def diagnose_vta29():
    raw_root = "data/raw"
    driver = "Vta (Driver E)"
    session_name = "Vta29"
    dt = 0.1

    print(f"=== Diagnosing {session_name} ===")
    s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
    synced = preprocess_session(s_df, v_df, target_dt=dt)

    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    # Calibration
    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
    print(f"Calib R_phone_to_veh:\n{calib.R_phone_to_veh}")
    print(f"Calib gyro_bias: {calib.gyro_bias}")
    print(f"Calib accel_bias: {calib.accel_bias}")

    if mag is not None:
        calib.calibrate_magnetometer(mag[:1200])
        print(f"Mag calibrated: {calib.mag_is_calibrated}, quality: {getattr(calib, 'mag_calibration_quality', 0.0)}")

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

    fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=1000.0)
    rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
    fusion.road_network = rn
    fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))
    eval_end = min(N, outage_end + int(10.0 / dt))

    print(f"Total points: {N}, Outage window: {outage_start} ({outage_start*dt:.1f}s) to {outage_end} ({outage_end*dt:.1f}s)")

    results = []
    outage_data = []

    for i in range(1, eval_end):
        in_outage = (outage_start <= i <= outage_end)

        if i == outage_start:
            if fusion.map_matcher is not None:
                fusion.map_matcher.reset_history()
            fusion._last_matched_point = None
            fusion._last_matched_time = None
            print(f"--- ENTERING OUTAGE at t={i*dt:.1f}s, speed_scale={fusion.speed_scale:.3f} ---")
            print(f"Pre-outage EKF pos: {fusion.ekf.p[:2]}, GT pos: {[e_gt[i], n_gt[i]]}")
            print(f"Pre-outage EKF vel: {fusion.ekf.v[:2]}, GT speed: {speed[i]:.2f}")
            print(f"Pre-outage EKF euler: {fusion.ekf.get_euler_angles_deg()}, GT heading: {gt_heading[i]:.2f}")
            print(f"Pre-outage gyro bias est in EKF: {fusion.ekf.b_g}")

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

        if in_outage:
            err = np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[i], n_gt[i]]))
            euler = fusion.ekf.get_euler_angles_deg()
            heading_err = ((euler[2] - gt_heading[i] + 180) % 360) - 180
            outage_data.append({
                "t": i * dt,
                "gt_speed": speed[i],
                "ai_speed_scaled": res["ai_speed"] * fusion.speed_scale if res["ai_speed"] is not None else 0,
                "ekf_speed": np.linalg.norm(fusion.ekf.v[:2]),
                "gt_heading": gt_heading[i],
                "ekf_heading": euler[2],
                "heading_err": heading_err,
                "pos_err": err,
                "mm_snapped": getattr(fusion._last_map_match, 'snapped', False) if hasattr(fusion, '_last_map_match') and fusion._last_map_match else False,
            })

    out_df = pd.DataFrame(outage_data)
    print("\n--- Outage Summary Samples ---")
    print(out_df.iloc[::50].to_string())

    final_err = out_df["pos_err"].iloc[-1]
    outage_dist = float(np.sum(np.sqrt(np.diff(e_gt[outage_start:outage_end+1])**2 + np.diff(n_gt[outage_start:outage_end+1])**2)))
    drift_pct = (final_err / outage_dist) * 100.0
    print(f"\nFinal Outage Error: {final_err:.2f} m, Dist: {outage_dist:.2f} m, Drift: {drift_pct:.2f}%")

if __name__ == "__main__":
    diagnose_vta29()
