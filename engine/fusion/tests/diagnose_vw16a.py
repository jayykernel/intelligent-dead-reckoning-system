import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

def diagnose_vw16a():
    driver = "Vw (Driver E)"
    raw_root = "data/raw"
    dt = 0.1
    s_df, v_df = load_iovnbd_session(raw_root, driver, "Vw16a")
    synced = preprocess_session(s_df, v_df, target_dt=dt)

    N = len(synced)
    print(f"Session Vw16a total samples: {N} ({N*dt:.2f} s)")

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:min(1200, N)], gyro[:min(1200, N)], speed[:min(1200, N)], dt=dt)
    if mag is not None:
        calib.calibrate_magnetometer(mag[:min(1200, N)])

    print(f"Calib gyro_bias: {calib.gyro_bias}")
    print(f"Calib accel_bias: {calib.accel_bias}")
    print(f"Calib mag_is_calibrated: {calib.mag_is_calibrated}, quality: {getattr(calib, 'mag_calibration_quality', 0.0)}")

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
    from engine.map_matching.hmm_matcher import HMMMapMatcher
    fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))
    eval_end = min(N, outage_end + int(10.0 / dt))

    print(f"outage_start: {outage_start} ({outage_start*dt:.1f}s), outage_end: {outage_end} ({outage_end*dt:.1f}s)")

    pre_outage_gnss_pos_eval = 0
    pre_outage_gnss_pos_acc = 0

    for i in range(1, eval_end):
        in_outage = (outage_start <= i <= outage_end)
        if i == outage_start:
            if fusion.map_matcher is not None:
                fusion.map_matcher.reset_history()
            fusion._last_matched_point = None
            fusion._last_matched_time = None
            ekf_euler = fusion.ekf.get_euler_angles_deg()
            print(f"--- Entering outage at step {i} ({i*dt:.1f}s) ---")
            print(f"Speed scale: {fusion.speed_scale:.4f}, updates count: {fusion._speed_scale_updates}")
            print(f"EKF pos: {fusion.ekf.p[:2]}, GT pos: {[e_gt[i], n_gt[i]]}, Pos error: {np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[i], n_gt[i]])):.2f}m")
            print(f"EKF vel: {fusion.ekf.v}, GT speed: {speed[i]:.2f}, EKF speed: {np.linalg.norm(fusion.ekf.v):.2f}")
            print(f"EKF Euler (deg): {ekf_euler}, GT Heading (deg): {gt_heading[i]:.2f}, Heading error: {((ekf_euler[2] - gt_heading[i] + 180)%360)-180:.2f} deg")

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

        if not in_outage and i < outage_start:
            pre_outage_gnss_pos_eval += 1
            if res["gnss_pos_passed"]:
                pre_outage_gnss_pos_acc += 1

        if in_outage:
            if (i - outage_start) % 50 == 0 or i == outage_end:
                err = np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[i], n_gt[i]]))
                cum_dist = np.sum(np.linalg.norm(np.diff(np.column_stack([e_gt[outage_start:i+1], n_gt[outage_start:i+1]]), axis=0), axis=1)) if i > outage_start else 0.0
                ekf_yaw = fusion.ekf.get_euler_angles_deg()[2]
                yaw_err = ((ekf_yaw - gt_heading[i] + 180) % 360) - 180
                mm_snap = getattr(fusion, '_last_map_match', None)
                conf = mm_snap.confidence if (mm_snap and hasattr(mm_snap, 'confidence')) else 0.0
                reason = getattr(mm_snap, 'fallback_reason', None) if mm_snap else None
                mm_info = f"MM snapped={mm_snap.snapped if mm_snap else False} (conf={conf:.2f}, reason={reason})" if mm_snap else "MM none"
                print(f"OUTAGE t={i*dt:.1f}s (+{(i-outage_start)*dt:.1f}s) | err={err:.2f}m dist={cum_dist:.2f}m drift={err/max(1e-3, cum_dist)*100:.1f}% | yaw_err={yaw_err:.1f}deg | ai_spd={res['ai_speed']:.2f} ekf_spd={np.linalg.norm(fusion.ekf.v):.2f} gt_spd={speed[i]:.2f} | {mm_info}")

    outage_dist = np.sum(np.linalg.norm(np.diff(np.column_stack([e_gt[outage_start:outage_end+1], n_gt[outage_start:outage_end+1]]), axis=0), axis=1))
    final_err = np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[outage_end], n_gt[outage_end]]))
    print(f"\nFINAL: Outage dist={outage_dist:.2f}m, Final err={final_err:.2f}m, Drift={final_err/outage_dist*100:.2f}%")
    print(f"Pre-outage GNSS pos acceptance: {pre_outage_gnss_pos_acc}/{pre_outage_gnss_pos_eval} ({pre_outage_gnss_pos_acc/max(1, pre_outage_gnss_pos_eval)*100:.1f}%)")

if __name__ == "__main__":
    diagnose_vw16a()
