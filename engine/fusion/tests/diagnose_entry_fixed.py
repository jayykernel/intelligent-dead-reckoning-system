import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import (
    latlon_to_enu, ProductionMobileFusionEngine, build_gt_road_network
)
from engine.map_matching.hmm_matcher import HMMMapMatcher

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

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

fusion = ProductionMobileFusionEngine(dt=0.1, default_vehicle_type="car", k=1000.0)
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

outage_start = 3000
outage_end = outage_start + 600
eval_end = outage_end + 100

print("Step-by-step tracking at outage entry:")
print(f"Outage from step {outage_start} to {outage_end}")
print()

for i in range(outage_start, min(outage_start + 10, eval_end)):
    in_outage = (outage_start <= i <= outage_end)

    if i == outage_start:
        if fusion.map_matcher is not None:
            fusion.map_matcher.reset_history()
        fusion._last_matched_point = None
        fusion._last_matched_time = None

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
        timestamp=i * 0.1
    )

    if i == outage_start:
        ekf_yaw = fusion.ekf.get_euler_angles_deg()[2]
        ekf_speed = np.linalg.norm(fusion.ekf.v)
        print(f"=== OUTAGE ENTRY (Step {i}) ===")
        print(f"GT Heading: {gt_heading[i]:+7.2f}° | EKF Heading: {ekf_yaw:+7.2f}° | Diff: {(ekf_yaw - gt_heading[i]):+7.2f}°")
        print(f"GT Speed: {speed[i]:6.2f} m/s | EKF Speed: {ekf_speed:6.2f} m/s")
        print(f"EKF Gyro Bias: {fusion.ekf.b_g}")
        print(f"EKF Accel Bias: {fusion.ekf.b_a}")
        print(f"EKF Position error: {np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[i], n_gt[i]])):7.2f} m")
        print()

    if in_outage and (i - outage_start) % 50 == 0:
        step_in_out = i - outage_start
        err = np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[i], n_gt[i]]))
        cur_yaw = fusion.ekf.get_euler_angles_deg()[2]
        print(f"Outage t=+{step_in_out*0.1:5.1f}s | Error: {err:7.1f}m | Yaw: {cur_yaw:+7.1f}° (GT: {gt_heading[i]:+7.1f}°)")

final_err = np.linalg.norm(fusion.ekf.p[:2] - np.array([e_gt[outage_end], n_gt[outage_end]]))
outage_dist = np.sum(np.sqrt(np.diff(e_gt[outage_start:outage_end+1])**2 + np.diff(n_gt[outage_start:outage_end+1])**2))
drift_pct = (final_err / outage_dist * 100.0) if outage_dist > 0 else 0.0

print()
print(f"Final Error: {final_err:.2f}m, Distance: {outage_dist:.2f}m, Drift: {drift_pct:.2f}%")
