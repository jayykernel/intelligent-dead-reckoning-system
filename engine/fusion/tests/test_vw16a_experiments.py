import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import (
    load_iovnbd_session,
    preprocess_session,
    latlon_to_enu,
    CalibrationEngine,
    ProductionMobileFusionEngine,
    build_gt_road_network,
    HMMMapMatcher
)

def run_sim(fixed_speed_scale=None, disable_mag=False, disable_map_matching=False):
    s_df, v_df = load_iovnbd_session("data/raw", "Vw (Driver E)", "Vw16a")
    synced = preprocess_session(s_df, v_df, target_dt=0.1)
    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)
    if mag is not None and not disable_mag:
        calib.calibrate_magnetometer(mag[:1200])

    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / 0.1)

    fusion = ProductionMobileFusionEngine(dt=0.1, default_vehicle_type="car", k=1000.0)
    if not disable_map_matching:
        rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
        fusion.road_network = rn
        fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")
    else:
        fusion.road_network = None
        fusion.map_matcher = None

    p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
    h_rad0 = np.radians(gt_heading[0])
    v0 = np.array([speed[0] * np.sin(h_rad0), speed[0] * np.cos(h_rad0), 0.0])
    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    outage_gt_pts = []
    outage_est_pts = []

    for i in range(1, outage_end + 1):
        in_outage = (outage_start <= i <= outage_end)
        if i == outage_start:
            if fusion.map_matcher is not None:
                fusion.map_matcher.reset_history()
            fusion._last_matched_point = None
            fusion._last_matched_time = None
            if fixed_speed_scale is not None:
                fusion.speed_scale = fixed_speed_scale

        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
        m_raw = mag[i] if (mag is not None and not disable_mag) else None

        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=m_raw,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage,
            timestamp=i * 0.1
        )
        if fixed_speed_scale is not None and in_outage:
            fusion.speed_scale = fixed_speed_scale

        if in_outage:
            outage_gt_pts.append([e_gt[i], n_gt[i]])
            outage_est_pts.append(fusion.ekf.p[:2].copy())

    outage_gt_pts = np.array(outage_gt_pts)
    outage_est_pts = np.array(outage_est_pts)
    gt_dist = np.sum(np.linalg.norm(np.diff(outage_gt_pts, axis=0), axis=1))
    final_err = np.linalg.norm(outage_est_pts[-1] - outage_gt_pts[-1])
    drift_pct = (final_err / gt_dist) * 100
    return gt_dist, final_err, drift_pct, getattr(fusion, 'speed_scale', 1.0)

print("1. Baseline (default):", run_sim())
print("2. Fixed speed_scale=1.0 (unbiased AI speed):", run_sim(fixed_speed_scale=1.0))
print("3. Fixed speed_scale=1.02 (ideal scale):", run_sim(fixed_speed_scale=1.02))
