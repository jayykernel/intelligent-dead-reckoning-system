import numpy as np
import os
import pandas as pd
from training.data_loader import load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import latlon_to_enu
from engine.map_matching.hmm_matcher import HMMMapMatcher

# Load session 2
raw_path = os.path.join("data/raw/two_wheeler")
synced = load_two_wheeler_session(raw_path, 'session2')
dt = 0.1
acc = synced[['acc_x', 'acc_y', 'acc_z']].values
gyro = synced[['gyro_x', 'gyro_y', 'gyro_z']].values
speed = np.nan_to_num(synced['gt_speed'].values, nan=0.0)
mag = synced[['mag_x', 'mag_y', 'mag_z']].values if 'mag_x' in synced.columns else None

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
if mag is not None:
    calib.calibrate_magnetometer(mag[:1200])

lat0 = synced['gt_lat'].iloc[0]
lon0 = synced['gt_lon'].iloc[0]
alt0 = synced['gt_alt'].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced['gt_lat'].values, synced['gt_lon'].values, synced['gt_alt'].values, lat0, lon0, alt0)

gt_heading = synced['gt_heading'].values
if np.any(np.isnan(gt_heading)):
    gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
heading_rad = np.radians(gt_heading[0])
v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type='two_wheeler', k=1000.0)
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type='two_wheeler')
fusion.map_matcher.search_radius = 80.0
fusion.map_matcher.heading_weight = 1.0
fusion.map_matcher.max_deviation_m = 100.0

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)
if calib.mag_is_calibrated:
    fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
    fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
    fusion.calib.mag_is_calibrated = True
    fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

N = len(synced)
outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / dt)

print(f"Total N={N}, Outage: {outage_start} to {outage_end} ({outage_start*dt:.1f}s to {outage_end*dt:.1f}s)")
print("Time | Mode | V_est | V_gt | AI_spd | Sc_AI | S_Scale | H_est | H_gt | D_err")

for i in range(1, outage_end + 50):
    in_outage = (outage_start <= i <= outage_end)
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
    m_raw = mag[i] if mag is not None else None

    if i == outage_start:
        fusion.map_matcher.reset_history()
        print("--- ENTERING OUTAGE ---")

    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=m_raw,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage,
        timestamp=i * dt
    )

    if (in_outage and i % 20 == 0) or (i == outage_start) or (i == outage_end):
        v_est_mag = np.linalg.norm(res["vel"][:2])
        p_est = res["pos"][:2]
        p_gt = np.array([e_gt[i], n_gt[i]])
        d_err = np.linalg.norm(p_est - p_gt)
        h_est = res["euler_deg"][2]
        ai_spd = res["ai_speed"]
        sc = res["speed_scale"]
        print(f"{i*dt:5.1f} | {res['mode'][:4]} | {v_est_mag:5.2f} | {speed[i]:5.2f} | {ai_spd:5.2f} | {ai_spd*sc:5.2f} | {sc:5.3f} | {h_est:6.1f} | {gt_heading[i]:6.1f} | {d_err:6.2f}m")
