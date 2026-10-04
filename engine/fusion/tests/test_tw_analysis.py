import numpy as np
import os
from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from engine.map_matching.hmm_matcher import HMMMapMatcher

synced = load_two_wheeler_session("data/raw/two_wheeler", "session2")
N = len(synced)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

dt = 0.1
calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
if mag is not None:
    calib.calibrate_magnetometer(mag[:1200])

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

fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="two_wheeler", k=1000.0)
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="two_wheeler")
fusion.map_matcher.search_radius = 150.0
fusion.map_matcher.heading_weight = 1.0
fusion.map_matcher.max_deviation_m = 100.0

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

if calib.mag_is_calibrated:
    fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
    fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
    fusion.calib.mag_is_calibrated = True
    fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + 600

print(f"Starting simulation. Outage: {outage_start} to {outage_end}")
for i in range(1, outage_end + 1):
    in_outage = (outage_start <= i <= outage_end)
    if i == outage_start:
        fusion.map_matcher.reset_history()
        print(f"Outage entered. Speed scale: {getattr(fusion, 'speed_scale', 1.0):.3f}")

    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    vel_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

    # Process
    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=mag[i] if mag is not None else None,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=vel_enu,
        is_gnss_available=not in_outage,
        timestamp=i * dt
    )

    if in_outage and (i - outage_start) % 50 == 0:
        p_est = fusion.ekf.p
        p_gt = np.array([e_gt[i], n_gt[i]])
        err = np.linalg.norm(p_est[:2] - p_gt)
        h_est = fusion.ekf.get_euler_angles_deg()[2]
        print(f"t={(i-outage_start)*dt:.1f}s | EKF: ({p_est[0]:.1f}, {p_est[1]:.1f}) | GT: ({p_gt[0]:.1f}, {p_gt[1]:.1f}) | Err: {err:.1f}m | H_est: {h_est:.1f} | H_gt: {gt_heading[i]:.1f}")
        # Print map match state
        last_seg = fusion.map_matcher.last_matched_seg
        if last_seg:
            print(f"   MM: seg={last_seg.segment_id} (bearing={last_seg.bearing_deg:.1f})")

p_final_est = fusion.ekf.p[:2]
p_final_gt = np.array([e_gt[outage_end], n_gt[outage_end]])
final_err = np.linalg.norm(p_final_est - p_final_gt)
path_len = np.sqrt(np.diff(e_gt[outage_start:outage_end])**2 + np.diff(n_gt[outage_start:outage_end])**2).sum()
print(f"\nFinal Error: {final_err:.2f}m / Path: {path_len:.2f}m -> Drift: {final_err/path_len*100:.2f}%")
