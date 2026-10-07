import numpy as np
import os
import sys
from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from engine.map_matching.hmm_matcher import HMMMapMatcher

def main():
    raw_path = "data/raw/two_wheeler"
    session_id = 'session1'
    synced = load_two_wheeler_session(raw_path, session_id)

    dt = 0.1
    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

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

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)
    if calib.mag_is_calibrated:
        fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
        fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
        fusion.calib.mag_is_calibrated = True
        fusion.calib.mag_calibration_quality = calib.mag_calibration_quality

    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))
    eval_end = min(N, outage_end + int(10.0 / dt))

    results = []
    outage_gt_pts = []

    for i in range(1, eval_end):
        in_outage = (outage_start <= i <= outage_end)

        if i == outage_start:
            if fusion.map_matcher is not None:
                fusion.map_matcher.reset_history()
            fusion._last_matched_point = None
            fusion._last_matched_time = None
            print(f'[{session_id}] Entering outage. speed_scale={getattr(fusion, "speed_scale", 1.0):.3f}')

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
            outage_gt_pts.append(np.array([e_gt[i], n_gt[i], u_gt[i]]))

    outage_est_pts = [r["pos"] for idx, r in enumerate(results) if outage_start <= (idx + 1) <= outage_end]
    final_pos_err = np.linalg.norm(outage_est_pts[-1][:2] - outage_gt_pts[-1][:2])
    dist_travelled = np.sum(np.linalg.norm(np.diff(outage_gt_pts, axis=0), axis=1))
    drift_pct = (final_pos_err / dist_travelled) * 100.0 if dist_travelled > 0 else 0.0

    print(f"Final Pos Err: {final_pos_err:.2f} m")
    print(f"Dist Travelled: {dist_travelled:.2f} m")
    print(f"Drift %: {drift_pct:.2f} %")

if __name__ == "__main__":
    main()
