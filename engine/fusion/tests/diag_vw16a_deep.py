import os
import sys
import numpy as np

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from eval.run_full_benchmark import evaluate_dead_reckoning_session, ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

raw_root = "data/raw"
s_df, v_df = load_iovnbd_session(raw_root, "Vw (Driver E)", "Vw16a")
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

# Replicate the logic from evaluate_dead_reckoning_session for Vw16a
N = len(synced)
outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / 0.1)
outage_end = min(outage_end, N - int(10.0 / 0.1))
eval_end = min(N, outage_end + int(10.0 / 0.1))

fusion = ProductionMobileFusionEngine(dt=0.1, default_vehicle_type="car", k=1000.0)
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
from engine.map_matching.hmm_matcher import HMMMapMatcher
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")

# Transfer magnetometer calibration
if calib.mag_is_calibrated:
    fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
    fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
    fusion.calib.mag_is_calibrated = True
    fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

speed_scales = []
has_nans = False
ai_speeds_pre = []
gnss_speeds_pre = []
dr_dist = 0.0
last_pos = p0
outage_gt_pts = []

print(f"Vw16a Outage window: {outage_start} to {outage_end} ({outage_end-outage_start} steps)")

for i in range(1, eval_end):
    in_outage = (outage_start <= i <= outage_end)

    # Ensure MapMatcher has a clean start right when GNSS drops
    if i == outage_start:
        if fusion.map_matcher is not None:
            fusion.map_matcher.reset_history()
        fusion._last_matched_point = None
        fusion._last_matched_time = None
        print(f'[Vw16a] Entering outage. speed_scale={getattr(fusion, "speed_scale", 1.0):.3f}')

    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
    m_raw = mag[i] if mag is not None else None

    # Track NaNs
    temp_fusion = ProductionMobileFusionEngine(dt=0.1, default_vehicle_type="car", k=1000.0)
    temp_fusion.road_network = rn
    temp_fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")
    if calib.mag_is_calibrated:
        temp_fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
        temp_fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
        temp_fusion.calib.mag_is_calibrated = True
        temp_fusion.calib.mag_calibration_quality = calib.mag_calibration_quality
    temp_fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    res = temp_fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=m_raw,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage,
        timestamp=i * 0.1
    )

    # Check for NaNs in EKF state (using correct attribute names)
    if np.any(np.isnan(temp_fusion.ekf.p)) or np.any(np.isnan(temp_fusion.ekf.P)):
        has_nans = True
        print(f"NaN Detected at step {i}!")
        break

    if i < outage_start and not in_outage:
        # Pre-outage: compare AI speed to GNSS speed
        acc_veh, gyro_veh = temp_fusion.calib.apply(acc[i], gyro[i])
        ai_s, _, _ = temp_fusion.ai_corrector.process_imu_sample(acc[i], gyro[i])
        gnss_s = np.linalg.norm(v_enu[:2]) if v_enu is not None else 0.0
        if ai_s is not None and gnss_s > 0.1:
            ai_speeds_pre.append(ai_s)
            gnss_speeds_pre.append(gnss_s)

    # Now do the actual step with the real fusion engine
    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=m_raw,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage,
        timestamp=i * 0.1
    )

    speed_scales.append(fusion.speed_scale)

    if in_outage:
        pos = fusion.ekf.p
        if i == outage_start:
            last_pos = pos
        dr_dist += np.linalg.norm(pos[:2] - last_pos[:2])
        last_pos = pos
        outage_gt_pts.append([e_gt[i], n_gt[i]])

print(f"Has NaNs: {has_nans}")
print(f"Final Speed Scale: {speed_scales[-1]:.4f}")
print(f"Speed scale min/max/mean: {np.min(speed_scales):.4f} / {np.max(speed_scales):.4f} / {np.mean(speed_scales):.4f}")

if ai_speeds_pre and gnss_speeds_pre:
    print(f"Mean AI speed (pre-outage): {np.mean(ai_speeds_pre):.2f}")
    print(f"Mean GNSS speed (pre-outage): {np.mean(gnss_speeds_pre):.2f}")

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

print(f"\n=== VW16A ANALYSIS RESULTS ===")
print(f"Simulated Outage Dist: {outage_dist:.2f} m")
print(f"Simulated Final Error: {final_err:.2f} m")
print(f"Simulated Drift %: {drift_pct:.2f} %")
print(f"Stationary? {is_stationary}")

# Compare with benchmark results
print(f"\n=== BENCHMARK RESULTS ===")
print(f"Benchmark Outage Dist: 1187.27 m")
print(f"Benchmark Final Error: 397.63 m")
print(f"Benchmark Drift %: 33.49 %")

print(f"\n=== DRIFT CALCULATION CHECK ===")
print(f"Final error / outage dist = {final_err*100/outage_dist:.2f}% (should match drift_pct)")
