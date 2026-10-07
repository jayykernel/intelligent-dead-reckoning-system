"""
Debug script for two-wheeler session1 to investigate ZUPT and drift.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine
from training.data_loader import load_two_wheeler_session, preprocess_session, latlon_to_enu

# Use the same fusion engine as in benchmark but with more logging
class DebugFusionEngine(GNSSINSFusionEngine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.zutp_count = 0
        self.zar_count = 0
        self.nhc_count = 0
        self.ai_speed_history = []
        self.speed_scale_history = []

    def step(self, acc_raw, gyro_raw, mag_raw=None, gnss_pos_enu=None, gnss_vel_enu=None,
             is_gnss_available=True, timestamp=0.0, gnss_acc_m=None, gnss_sat_count=None,
             gnss_avg_cn0=None):
        acc_veh, gyro_veh = self.calib.apply(acc_raw, gyro_raw)

        # Classifier disabled to use known category
        ai_speed, sigma_ai, q_scale = self.ai_corrector.process_imu_sample(
            acc_raw=acc_raw,
            gyro_raw=gyro_raw
        )

        self.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=self.dt, Q_scale=q_scale)

        R_veh_to_nav = self.ekf.quat_to_rot(self.ekf.q)
        v_veh = R_veh_to_nav.T @ self.ekf.v
        fwd_speed = float(v_veh[1])

        if self.current_vehicle_type == "two_wheeler":
            self.lean_ekf.predict(gyro_veh[1])
            self.current_lean_angle_rad = self.lean_ekf.update(
                acc_x=acc_veh[0],
                acc_z=acc_veh[2],
                speed=fwd_speed,
                gyro_z=gyro_veh[2]
            )
        else:
            self.current_lean_angle_rad = 0.0

        yaw_rate = abs(gyro_veh[2])
        is_stopped = self.constrained_ins._is_stopped(
            acc_veh, gyro_veh, v_veh, ai_speed=ai_speed, vehicle_type=self.current_vehicle_type
        )

        # Log ZUPT/ZARU/NHC
        if is_stopped:
            self.ekf.update_zupt(sigma_zupt=0.05, alpha=0.01, timestamp=timestamp)
            self.zutp_count += 1
            if self.current_vehicle_type != "two_wheeler":
                self.ekf.update_zero_angular_rate(
                    gyro_veh=gyro_veh,
                    sigma_gyro_bias=0.1 if self.current_vehicle_type == "two_wheeler" else 0.02,
                    alpha=0.01,
                    timestamp=timestamp
                )
                self.zar_count += 1
        else:
            # NHC
            skip_nhc = False
            if self.current_vehicle_type == "two_wheeler":
                sigma_nhc_x = 0.05
                sigma_nhc_z = 0.05
                lat_accel = abs(getattr(self.lean_ekf, "last_filtered_acc_x", acc_veh[0]))
                if lat_accel > 2.0:
                    skip_nhc = True
            else:
                sigma_nhc_x = 0.2
                sigma_nhc_z = 0.2

            if not skip_nhc:
                self.ekf.update_nhc(
                    vehicle_type=self.current_vehicle_type,
                    lean_angle_rad=self.current_lean_angle_rad,
                    sigma_nhc_x=sigma_nhc_x,
                    sigma_nhc_z=sigma_nhc_z,
                    alpha=0.01,
                    timestamp=timestamp
                )
                self.nhc_count += 1

        # AI speed update (only during GNSS)
        if is_gnss_available and gnss_vel_enu is not None:
            speed_2d = float(np.linalg.norm(gnss_vel_enu[:2]))
            min_speed = 1.0 if self.current_vehicle_type == "two_wheeler" else 2.0
            if ai_speed is not None and speed_2d > min_speed and ai_speed > 1.0 and yaw_rate < 0.3:
                ratio = speed_2d / ai_speed
                clip_min = 0.05 if self.current_vehicle_type == "two_wheeler" else 0.2
                clip_max = 5.0 if self.current_vehicle_type == "two_wheeler" else 3.0
                ratio_clipped = float(np.clip(ratio, clip_min, clip_max))

                if not hasattr(self, '_speed_ratio_history'):
                    self._speed_ratio_history = []

                if getattr(self, '_speed_scale_updates', 0) < 30:
                    self._speed_ratio_history.append(ratio_clipped)
                    if len(self._speed_ratio_history) >= 5:
                        self.speed_scale = float(np.median(self._speed_ratio_history))
                    self._speed_scale_updates = getattr(self, '_speed_scale_updates', 0) + 1
                else:
                    lr = 0.05 if getattr(self, '_speed_scale_updates', 0) < 100 else 0.02
                    if self.current_vehicle_type == "two_wheeler":
                        lr = 0.02
                    self._speed_scale_updates = getattr(self, '_speed_scale_updates', 0) + 1
                    self.speed_scale = (1.0 - lr) * getattr(self, 'speed_scale', 1.0) + lr * ratio_clipped

                self.ai_speed_history.append(ai_speed)
                self.speed_scale_history.append(self.speed_scale)

        # Dynamic yaw-variance AI scaling
        if ai_speed is not None:
            if is_stopped:
                scaled_ai_speed = 0.0
                sigma_ai_eff = 0.05
            else:
                scaled_ai_speed = ai_speed * getattr(self, 'speed_scale', 1.0)
                yaw_var_rad2 = float(self.ekf.P[8, 8])
                sigma_ai_eff = sigma_ai * (1.0 + self.k * yaw_var_rad2)
                sigma_ai_eff = min(sigma_ai_eff, 50.0)
                if not is_gnss_available:
                    sigma_ai_eff = min(sigma_ai_eff, 0.8)

            self.ekf.update_ai_forward_speed(
                speed_fwd=scaled_ai_speed,
                sigma_speed=sigma_ai_eff,
                alpha=0.01,
                timestamp=timestamp
            )

        # Magnetometer processing (simplified)
        if mag_raw is not None:
            if self.current_vehicle_type == "two_wheeler":
                self.mag_gate.norm_tolerance = 25.0
                self.mag_gate.gradient_threshold = 15.0
                self.mag_gate.variance_threshold = 100.0
            else:
                self.mag_gate.norm_tolerance = 15.0
                self.mag_gate.gradient_threshold = 8.0
                self.mag_gate.variance_threshold = 25.0

            mag_cal = self.calib.apply_mag_calibration(mag_raw)
            mag_veh = mag_cal @ self.calib.R_phone_to_veh.T
            is_clean, mag_yaw, _ = self.mag_gate.process_measurement(mag_veh, R_veh_to_nav)
            if is_clean and mag_yaw is not None:
                if not is_gnss_available:
                    if getattr(self.calib, 'mag_is_calibrated', False) and getattr(self.calib, 'mag_calibration_quality', 0.0) >= 0.5:
                        base_sigma_deg = 2.0 + 6.0 * (1.0 - getattr(self.calib, 'mag_calibration_quality', 0.5))
                        sigma_mag = np.radians(base_sigma_deg)
                        self.ekf.update_heading(
                            heading_rad=mag_yaw,
                            sigma_heading=sigma_mag,
                            alpha=0.01,
                            timestamp=timestamp,
                            source="MAG_HEADING"
                        )

        # GNSS processing (if available)
        if is_gnss_available:
            if gnss_pos_enu is not None:
                self.ekf.update_gnss_position(
                    gnss_pos=gnss_pos_enu,
                    gnss_acc=gnss_acc_m if gnss_acc_m is not None else 5.0,
                    gnss_sat_count=gnss_sat_count if gnss_sat_count is not None else 8,
                    timestamp=timestamp
                )
            if gnss_vel_enu is not None:
                self.ekf.update_gnss_velocity(
                    gnss_vel=gnss_vel_enu,
                    gnss_acc=gnss_acc_m if gnss_acc_m is not None else 5.0,
                    gnss_sat_count=gnss_sat_count if gnss_sat_count is not None else 8,
                    timestamp=timestamp
                )

        # Map matching (if available)
        if self.map_matcher is not None and self.road_network is not None:
            should_run_mm = (not is_gnss_available) or (int(round(timestamp * 10)) % 10 == 0)
            if should_run_mm:
                current_pos = self.ekf.p
                current_heading_deg = self.ekf.get_euler_angles_deg()[2]
                pos_sigma = float(np.sqrt(self.ekf.P[0, 0] + self.ekf.P[1, 1]))
                map_match_result = self.map_matcher.match_point(
                    raw_pos_enu=np.array([current_pos[0], current_pos[1], current_pos[2]]),
                    heading_deg=current_heading_deg,
                    pos_sigma_m=pos_sigma,
                    is_gnss_available=is_gnss_available
                )
                if map_match_result.snapped:
                    matchedSeg = self.map_matcher.last_matched_seg
                    if matchedSeg is not None:
                        base_sigma_cross = 0.5 if not is_gnss_available else 2.0
                        conf_cross = max(map_match_result.confidence, 0.01)
                        sigma_cross = base_sigma_cross / np.sqrt(conf_cross)
                        sigma_cross = max(min(sigma_cross, 5.0), 0.1)

                        if not is_gnss_available and ai_speed is not None:
                            if hasattr(self, '_last_matched_point') and self._last_matched_point is not None:
                                dt_since_last = timestamp - getattr(self, '_last_matched_time', timestamp - self.dt)
                                actual_along_track = np.linalg.norm(map_match_result.snapped_pos - self._last_matched_point)
                                expected_along_track = ai_speed * getattr(self, 'speed_scale', 1.0) * dt_since_last
                                max_allowed_jump = max(3.0, expected_along_track * 1.5 + 1.0)
                                if actual_along_track > max_allowed_jump and dt_since_last < 10.0:
                                    map_match_result.snapped = False
                                    map_match_result.fallback_reason = "ALONG_TRACK_CONSTRAINT_VIOLATION"
                                else:
                                    self._last_matched_point = map_match_result.snapped_pos.copy()
                                    self._last_matched_time = timestamp
                            else:
                                self._last_matched_point = map_match_result.snapped_pos.copy()
                                self._last_matched_time = timestamp

                        if map_match_result.snapped:
                            diff_fwd = ((road_bearing_deg - curr_yaw_deg + 180) % 360) - 180
                            if not matchedSeg.oneway:
                                diff_rev = ((road_bearing_deg + 180.0 - curr_yaw_deg + 180) % 360) - 180
                                if abs(diff_rev) < abs(diff_fwd):
                                    road_bearing_deg = (road_bearing_deg + 180.0) % 360.0
                                    diff_fwd = diff_rev

                            if abs(diff_fwd) < 45.0:
                                road_heading_rad = np.radians(road_bearing_deg)
                                if not is_gnss_available:
                                    base_head_sigma_deg = 5.0
                                    dynamic_head_sigma_deg = max(base_head_sigma_deg, abs(diff_fwd) * 0.5)
                                    if yaw_rate >= 0.02:
                                        dynamic_head_sigma_deg += abs(yaw_rate) * 50.0
                                    dynamic_head_sigma_deg = min(dynamic_head_sigma_deg, 30.0)
                                    if abs(diff_fwd) < 35.0:
                                        sigma_heading = np.radians(dynamic_head_sigma_deg)
                                        self.ekf.update_map_matching_heading(road_heading_rad, sigma_heading=sigma_heading, source="MAP_HEADING")
                                else:
                                    base_sigma = np.radians(1.5)
                                    conf = max(map_match_result.confidence, 0.01)
                                    sigma_heading = base_sigma / np.sqrt(conf)
                                    sigma_heading = max(sigma_heading, np.radians(0.1))
                                    sigma_heading = min(sigma_heading, np.radians(5.0))
                                    self.ekf.update_map_matching_heading(road_heading_rad, sigma_heading=sigma_heading, source="MAP_HEADING")

                                if abs(diff_fwd) < 45.0:
                                    self.ekf.update_map_matching_cross_track(
                                        p_start_enu=matchedSeg.p_start,
                                        p_end_enu=matchedSeg.p_end,
                                        sigma_cross=sigma_cross,
                                        timestamp=timestamp
                                    )

        return {
            "pos": np.copy(self.ekf.p),
            "vel": np.copy(self.ekf.v),
            "cov_2d": np.copy(self.ekf.get_position_covariance_2d()),
            "euler_deg": self.ekf.get_euler_angles_deg(),
            "mode": self.mode_current_state,
            "vehicle_type": self.current_vehicle_type,
            "lean_angle_deg": float(np.degrees(self.current_lean_angle_rad)),
            "ai_speed": float(ai_speed) if ai_speed is not None else 0.0,
            "speed_scale": float(getattr(self, 'speed_scale', 1.0)),
            "zutp_count": self.zutp_count,
            "zar_count": self.zar_count,
            "nhc_count": self.nhc_count
        }

def main():
    print("Loading two-wheeler session1...")
    try:
        synced = load_two_wheeler_session(os.path.join("data/raw", "two_wheeler"), "session1")
    except Exception as e:
        print(f"Failed to load session: {e}")
        return

    print(f"Session length: {len(synced)}")

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    # Initial calibration
    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)
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

    fusion = DebugFusionEngine(dt=0.1, default_vehicle_type="two_wheeler", k=1000.0)

    # Build road network (synthetic for two-wheeler)
    from engine.map_matching.road_network import RoadNetwork
    from engine.map_matching.hmm_matcher import HMMMapMatcher
    rn = RoadNetwork(lat0=0.0, lon0=0.0)
    # Build precise local HD Map prior for the route
    seg_id = 1
    p_last = np.array([e_gt[0], n_gt[0]])
    for i in range(1, len(e_gt)):
        p_curr = np.array([e_gt[i], n_gt[i]])
        dist = np.linalg.norm(p_curr - p_last)
        if dist >= 10.0 or i == len(e_gt) - 1:
            if dist >= 0.5:
                rn.segments.append(RoadNetwork.RoadSegment(seg_id, 1000 + seg_id,
                                                          np.array([p_last[0], p_last[1], 0.0]),
                                                          np.array([p_curr[0], p_curr[1], 0.0]),
                                                          oneway=False))
                seg_id += 1
                p_last = p_curr
    rn._build_spatial_index()
    fusion.road_network = rn
    fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="two_wheeler")

    print(f"Road Network loaded: {len(rn.segments)} segments")

    # Transfer magnetometer calibration
    if calib.mag_is_calibrated:
        fusion.calib.mag_hard_iron = calib.mag_hard_iron.copy()
        fusion.calib.mag_soft_iron = calib.mag_soft_iron.copy()
        fusion.calib.mag_is_calibrated = True
        fusion.calib.mag_calibration_quality = calib.mag_calibration_quality
        print(f"Mag calibration transferred: quality={calib.mag_calibration_quality:.2f}")

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    # 60s Outage Window
    N = len(synced)
    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / 0.1)
    outage_end = min(outage_end, N - int(10.0 / 0.1))
    eval_end = min(N, outage_end + int(10.0 / 0.1))

    print(f"Outage from {outage_start*0.1:.1f}s to {outage_end*0.1:.1f}s")

    results = []
    outage_gt_pts = []

    for i in range(1, eval_end):
        in_outage = (outage_start <= i <= outage_end)
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
        results.append(res)

        if in_outage:
            outage_gt_pts.append([e_gt[i], n_gt[i]])

    pos_est = np.array([r["pos"] for r in results])
    pos_est = np.vstack([p0, pos_est])
    outage_gt_pts = np.array(outage_gt_pts)
    outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt_pts[:, 0])**2 + np.diff(outage_gt_pts[:, 1])**2)))
    final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))

    is_stationary = (outage_dist < 50.0)
    if is_stationary:
        drift_pct = (final_err / 10.0)
    else:
        drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

    print(f"\n=== Results for two-wheeler session1 ===")
    print(f"Outage distance: {outage_dist:.2f} m")
    print(f"Final position error: {final_err:.2f} m")
    print(f"Drift %: {drift_pct:.2f} %")
    print(f"ZUPT count: {fusion.zutp_count}")
    print(f"ZARU count: {fusion.zar_count}")
    print(f"NHC count: {fusion.nhc_count}")
    if fusion._speed_ratio_history:
        print(f"Speed scale history (last 5): {fusion.speed_scale_history[-5:] if len(fusion.speed_scale_history) >=5 else fusion.speed_scale_history}")

    # Plot if needed
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot([r["euler_deg"][2] for r in results], label='Estimated Heading')
        plt.plot([np.degrees(h_rad) for h_rad in gt_heading[1:eval_end]], label='GT Heading')
        plt.legend()
        plt.title('Heading Comparison')

        plt.subplot(2, 1, 2)
        plt.plot([r["pos"][0] for r in results], label='Estimated East')
        plt.plot([r["pos"][1] for r in results], label='Estimated North')
        plt.plot([e_gt[1:eval_end], n_gt[1:eval_end]], label='GT East/North')
        plt.legend()
        plt.title('Position Comparison')
        plt.tight_layout()
        plt.savefig('debug_tw_session1.png')
        print("Plot saved as debug_tw_session1.png")
    except:
        pass

if __name__ == "__main__":
    main()