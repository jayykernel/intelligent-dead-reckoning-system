import os
import sys
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from engine.map_matching.hmm_matcher import HMMMapMatcher
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

def analyze_s4():
    raw_root = "data/raw"
    driver = "D1" # D1 or check dataset splits
    from training.dataset_splits import TEST_SESSIONS
    for d, s in TEST_SESSIONS:
        if s == "S4":
            driver = d
            break

    print(f"Loading S4 from driver {driver}...")
    s_df, v_df = load_iovnbd_session(raw_root, driver, "S4")
    dt = 0.1
    synced = preprocess_session(s_df, v_df, target_dt=dt)

    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
    print(f"R_phone_to_veh:\n{calib.R_phone_to_veh}")
    print(f"Gyro bias: {calib.gyro_bias}")
    print(f"Accel bias: {calib.accel_bias}")
    if mag is not None:
        mag_cal_ok = calib.calibrate_magnetometer(mag[:1200])
        print(f"Mag Calibrated: {mag_cal_ok}, Quality: {calib.mag_calibration_quality}")
        print(f"Mag Hard Iron: {calib.mag_hard_iron}")
        print(f"Mag Soft Iron:\n{calib.mag_soft_iron}")

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

    print(f"Outage indices: {outage_start} to {outage_end} (time: {outage_start*dt:.1f}s to {outage_end*dt:.1f}s)")

    # Store detailed step-by-step logs during outage
    outage_logs = []

    for i in range(1, eval_end):
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
            timestamp=i * dt
        )

        if in_outage:
            est_p = res["pos"]
            gt_p = np.array([e_gt[i], n_gt[i], u_gt[i]])
            pos_err = np.linalg.norm(est_p[:2] - gt_p[:2])

            est_yaw = res["euler_deg"][2]
            gt_yaw = gt_heading[i]
            yaw_err = ((est_yaw - gt_yaw + 180) % 360) - 180

            # Decompose error into along-track and cross-track
            # Unit vector along GT heading
            gt_h_rad = np.radians(gt_yaw)
            fwd_vec = np.array([np.sin(gt_h_rad), np.cos(gt_h_rad)]) # East, North
            right_vec = np.array([np.cos(gt_h_rad), -np.sin(gt_h_rad)]) # East, North

            delta_pos = est_p[:2] - gt_p[:2]
            along_err = np.dot(delta_pos, fwd_vec)
            cross_err = np.dot(delta_pos, right_vec)

            last_mm = getattr(fusion, "_last_map_match", None)
            snapped = last_mm.snapped if last_mm is not None else False
            fb_reason = getattr(last_mm, "fallback_reason", "") if last_mm is not None else ""

            outage_logs.append({
                "t": i * dt,
                "step": i - outage_start,
                "pos_err": pos_err,
                "along_err": along_err,
                "cross_err": cross_err,
                "est_yaw": est_yaw,
                "gt_yaw": gt_yaw,
                "yaw_err": yaw_err,
                "speed_gt": speed[i],
                "ai_speed": res["ai_speed"],
                "speed_scale": res["speed_scale"],
                "snapped": snapped,
                "fb_reason": fb_reason,
                "bias_g": np.copy(fusion.ekf.b_g),
                "bias_a": np.copy(fusion.ekf.b_a),
            })

    # Summary statistics
    total_dist = np.sum(np.sqrt(np.diff(e_gt[outage_start:outage_end])**2 + np.diff(n_gt[outage_start:outage_end])**2))
    final_log = outage_logs[-1]

    print("\n--- OUTAGE ANALYSIS ---")
    print(f"Total distance: {total_dist:.2f} m")
    print(f"Final Pos Error: {final_log['pos_err']:.2f} m (Along: {final_log['along_err']:.2f} m, Cross: {final_log['cross_err']:.2f} m)")
    print(f"Drift %: {final_log['pos_err'] / total_dist * 100:.2f}%")
    print(f"Final Yaw Error: {final_log['yaw_err']:.2f} deg")
    print(f"Speed scale at start of outage: {outage_logs[0]['speed_scale']:.4f}, at end: {final_log['speed_scale']:.4f}")

    # Check map matching snap rate
    snapped_count = sum(1 for l in outage_logs if l["snapped"])
    print(f"Map Matching Snapped Rate during outage: {snapped_count}/{len(outage_logs)} ({snapped_count/len(outage_logs)*100:.1f}%)")

    # Reason counts
    from collections import Counter
    reasons = Counter(l["fb_reason"] for l in outage_logs if not l["snapped"])
    print(f"Fallback reasons when not snapped: {dict(reasons)}")

    # Speed integration comparison
    gt_int_dist = np.sum([l["speed_gt"] * dt for l in outage_logs])
    ai_raw_int_dist = np.sum([l["ai_speed"] * dt for l in outage_logs])
    ai_scaled_int_dist = np.sum([l["ai_speed"] * l["speed_scale"] * dt for l in outage_logs])
    print(f"Integrated GT dist: {gt_int_dist:.2f} m")
    print(f"Integrated Raw AI dist: {ai_raw_int_dist:.2f} m (Error: {ai_raw_int_dist - gt_int_dist:.2f} m)")
    print(f"Integrated Scaled AI dist: {ai_scaled_int_dist:.2f} m (Error: {ai_scaled_int_dist - gt_int_dist:.2f} m)")

    print("\nDetailed Turn Snapshot (180s to 202s):")
    print("Time | AlongErr | CrossErr | PosErr | YawErr | Snapped | FB Reason | SpeedGT | AISpeed | GTYaw | EstYaw")
    for l in outage_logs:
        if 180.0 <= l['t'] <= 202.0 and int(round(l['t'] * 10)) % 10 == 0:
            fb_str = str(l.get('fb_reason') or '')[:15]
            print(f"{l['t']:5.1f}s | {l['along_err']:8.2f}m | {l['cross_err']:8.2f}m | {l['pos_err']:6.2f}m | {l['yaw_err']:6.2f} deg | {str(l['snapped']):7} | {fb_str:15} | {l['speed_gt']:7.2f} | {l['ai_speed']:7.2f} | {l['gt_yaw']:6.1f} | {l['est_yaw']:6.1f}")

if __name__ == '__main__':
    analyze_s4()
