"""
engine/fusion/tests/test_mode_aware_ai.py

Test mode-aware AI speed correction scaling during PURE_DEAD_RECKONING.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.fusion.fusion_engine import GNSSINSFusionEngine
from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

def eval_session(data_type, driver, session, sigma_scale_outage, disable_in_outage=False):
    raw_root = "data/raw"
    if data_type == "car":
        s_df, v_df = load_iovnbd_session(raw_root, driver, session)
        synced = preprocess_session(s_df, v_df, target_dt=0.1)
    else:
        synced = load_two_wheeler_session(os.path.join(raw_root, "two_wheeler"), session)

    N = len(synced)
    dt = 0.1

    calib = CalibrationEngine()
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = synced["gt_speed"].values
    speed = np.nan_to_num(speed, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

    veh_type = "two_wheeler" if data_type == "tw" else "car"
    fusion = GNSSINSFusionEngine(dt=dt, default_vehicle_type=veh_type)

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

    if session == "Vta26":
        outage_start = 1290
    else:
        outage_start = int(N * 0.4)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))

    results = []
    outage_path = []

    for i in range(1, N):
        in_outage = (outage_start <= i <= outage_end)

        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
        m_raw = mag[i] if mag is not None else None

        # Custom step logic with mode-aware sigma_ai
        # Step start:
        acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])

        if fusion.classifier is not None:
            feat_sample = np.hstack([acc_veh, gyro_veh])
            fusion.classifier_buffer.append(feat_sample)
            if len(fusion.classifier_buffer) > fusion.classifier_window_size:
                fusion.classifier_buffer.pop(0)
            if len(fusion.classifier_buffer) == fusion.classifier_window_size:
                acc_win = np.array([f[:3] for f in fusion.classifier_buffer])
                gyro_win = np.array([f[3:] for f in fusion.classifier_buffer])
                fusion.current_vehicle_type = fusion.classifier.predict_window(acc_win, gyro_win)

        ai_speed, sigma_ai, q_scale = fusion.ai_corrector.process_imu_sample(
            acc_raw=acc[i],
            gyro_raw=gyro[i]
        )

        fusion.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=dt, Q_scale=q_scale)

        R_veh_to_nav = fusion.ekf.quat_to_rot(fusion.ekf.q)
        v_veh = R_veh_to_nav.T @ fusion.ekf.v
        fwd_speed = float(v_veh[1])

        if fusion.current_vehicle_type == "two_wheeler":
            fusion.current_lean_angle_rad = fusion.lean_ekf.update(
                acc_x=acc_veh[0], acc_z=acc_veh[2], speed=fwd_speed, gyro_z=gyro_veh[2]
            )
        else:
            fusion.current_lean_angle_rad = 0.0

        is_stopped = fusion.constrained_ins._is_stopped(acc_veh, gyro_veh, v_veh)
        if is_stopped:
            fusion.ekf.update_zupt(sigma_zupt=0.05, alpha=0.01, timestamp=i*dt)
        else:
            fusion.ekf.update_nhc(
                vehicle_type=fusion.current_vehicle_type,
                lean_angle_rad=fusion.current_lean_angle_rad,
                sigma_nhc_x=0.2, sigma_nhc_z=0.2, alpha=0.01, timestamp=i*dt
            )

        # MODE-AWARE AI SPEED INJECTION
        is_pdr = (fusion.mode_current_state == "PURE_DEAD_RECKONING" or in_outage)
        if ai_speed is not None:
            if is_pdr:
                if not disable_in_outage:
                    eff_sigma = sigma_ai * sigma_scale_outage
                    fusion.ekf.update_ai_forward_speed(
                        speed_fwd=ai_speed, sigma_speed=eff_sigma, alpha=0.01, timestamp=i*dt
                    )
            else:
                fusion.ekf.update_ai_forward_speed(
                    speed_fwd=ai_speed, sigma_speed=sigma_ai, alpha=0.01, timestamp=i*dt
                )

        # GNSS Updates
        if not in_outage:
            trust_score = fusion.outage_predictor.update(avg_cn0=None, sat_count=None, accuracy_m=None)
        else:
            trust_score = 0.0

        fusion.mode_time_in_state += dt
        if fusion.mode_current_state == "GNSS_AIDED":
            if trust_score < fusion.mode_low_trust_threshold and fusion.mode_time_in_state >= fusion.mode_min_time_in_state:
                fusion.mode_current_state = "PURE_DEAD_RECKONING"
                fusion.mode_time_in_state = 0.0
        else:
            if trust_score > fusion.mode_high_trust_threshold and fusion.mode_time_in_state >= fusion.mode_min_time_in_state:
                fusion.mode_current_state = "GNSS_AIDED"
                fusion.mode_time_in_state = 0.0

        if fusion.mode_current_state == "GNSS_AIDED" and not in_outage and pos_enu is not None:
            dynamic_sigma_pos = 5.0 / max(0.1, np.sqrt(trust_score))
            fusion.ekf.update_gnss_position(p_gnss_enu=pos_enu, sigma_pos=dynamic_sigma_pos, alpha=0.01, timestamp=i*dt)
            if v_enu is not None:
                dynamic_sigma_vel = 0.5 / max(0.2, trust_score)
                fusion.ekf.update_gnss_velocity(v_gnss_enu=v_enu, sigma_vel=dynamic_sigma_vel, alpha=0.01, timestamp=i*dt)
                speed_2d = np.linalg.norm(v_enu[:2])
                if speed_2d >= 1.0:
                    sigma_h = np.radians(3.0) if speed_2d >= 1.5 else np.radians(3.0 + 7.0 * (1.5 - speed_2d) / 0.5)
                    cog_h = float(np.arctan2(v_enu[0], v_enu[1]))
                    fusion.ekf.update_heading(heading_rad=cog_h, sigma_heading=sigma_h, alpha=0.01, timestamp=i*dt, source="GNSS_HEADING")

        results.append(np.copy(fusion.ekf.p))
        if in_outage:
            outage_path.append([e_gt[i], n_gt[i]])

    pos_est = np.vstack([p0, np.array(results)])
    outage_path = np.array(outage_path)
    outage_dist = float(np.sum(np.sqrt(np.diff(outage_path[:, 0])**2 + np.diff(outage_path[:, 1])**2)))
    final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))
    drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

    return drift_pct, final_err, outage_dist

if __name__ == "__main__":
    configs = [
        ("1.0x (Uniform Continuous)", 1.0, False),
        ("3.0x in PDR", 3.0, False),
        ("5.0x in PDR", 5.0, False),
        ("10.0x in PDR", 10.0, False),
        ("Disabled in PDR (GNSS-aided only)", 1.0, True),
    ]

    sessions = [
        ("Car S4", "car", "S (Driver A)", "S4"),
        ("Car S1", "car", "S (Driver A)", "S1"),
        ("TW Sess 1", "tw", "S (Driver A)", "session1"),
        ("TW Sess 2", "tw", "S (Driver A)", "session2"),
    ]

    print("=== Mode-Aware AI Speed Experiment ===")
    for label, scale, dis in configs:
        print(f"\n--- Configuration: {label} ---")
        for s_name, dtype, drv, s_id in sessions:
            drift, err, dist = eval_session(dtype, drv, s_id, scale, dis)
            print(f"  {s_name:12s}: Drift = {drift:10.2f}% (Err = {err:10.2f}m, Dist = {dist:.1f}m)")
