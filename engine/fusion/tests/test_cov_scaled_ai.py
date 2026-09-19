"""
engine/fusion/tests/test_cov_scaled_ai.py

Test AI speed correction scaling based on EKF yaw variance.
"""

import os
import sys
import numpy as np
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.fusion.fusion_engine import GNSSINSFusionEngine
from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

class CovScaledFusionEngine(GNSSINSFusionEngine):
    def __init__(self, k=100.0, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.k = k  # scaling gain

    def step(self,
             acc_raw: np.ndarray,
             gyro_raw: np.ndarray,
             mag_raw: Optional[np.ndarray] = None,
             gnss_pos_enu: Optional[np.ndarray] = None,
             gnss_vel_enu: Optional[np.ndarray] = None,
             is_gnss_available: bool = True,
             timestamp: float = 0.0,
             gnss_acc_m: Optional[float] = None,
             gnss_sat_count: Optional[int] = None,
             gnss_avg_cn0: Optional[float] = None):

        acc_veh, gyro_veh = self.calib.apply(acc_raw, gyro_raw)

        if self.classifier is not None:
            feat_sample = np.hstack([acc_veh, gyro_veh])
            self.classifier_buffer.append(feat_sample)
            if len(self.classifier_buffer) > self.classifier_window_size:
                self.classifier_buffer.pop(0)

            if len(self.classifier_buffer) == self.classifier_window_size:
                acc_win = np.array([f[:3] for f in self.classifier_buffer])
                gyro_win = np.array([f[3:] for f in self.classifier_buffer])
                self.current_vehicle_type = self.classifier.predict_window(acc_win, gyro_win)

        ai_speed, sigma_ai, q_scale = self.ai_corrector.process_imu_sample(
            acc_raw=acc_raw,
            gyro_raw=gyro_raw
        )

        self.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=self.dt, Q_scale=q_scale)

        R_veh_to_nav = self.ekf.quat_to_rot(self.ekf.q)
        v_veh = R_veh_to_nav.T @ self.ekf.v
        fwd_speed = float(v_veh[1])

        if self.current_vehicle_type == "two_wheeler":
            self.current_lean_angle_rad = self.lean_ekf.update(
                acc_x=acc_veh[0],
                acc_z=acc_veh[2],
                speed=fwd_speed,
                gyro_z=gyro_veh[2]
            )
        else:
            self.current_lean_angle_rad = 0.0

        is_stopped = self.constrained_ins._is_stopped(acc_veh, gyro_veh, v_veh)
        if is_stopped:
            self.ekf.update_zupt(
                sigma_zupt=0.05,
                alpha=0.01,
                timestamp=timestamp
            )
        else:
            self.ekf.update_nhc(
                vehicle_type=self.current_vehicle_type,
                lean_angle_rad=self.current_lean_angle_rad,
                sigma_nhc_x=0.2,
                sigma_nhc_z=0.2,
                alpha=0.01,
                timestamp=timestamp
            )

        # Continuous covariance-based scaling
        if ai_speed is not None:
            # P[8, 8] is yaw variance (in rad^2)
            yaw_var_rad2 = float(self.ekf.P[8, 8])
            yaw_std_deg = np.degrees(np.sqrt(max(1e-8, yaw_var_rad2)))
            # We scale sigma_ai dynamically: when yaw_std is high, trust is lower
            # e.g., sigma_ai_eff = sigma_ai * (1.0 + k * yaw_var_rad2)
            sigma_ai_eff = sigma_ai * (1.0 + self.k * yaw_var_rad2)
            self.ekf.update_ai_forward_speed(
                speed_fwd=ai_speed,
                sigma_speed=sigma_ai_eff,
                alpha=0.01,
                timestamp=timestamp
            )

        gnss_pos_passed = False
        gnss_vel_passed = False

        if is_gnss_available:
            trust_score = self.outage_predictor.update(
                avg_cn0=gnss_avg_cn0,
                sat_count=gnss_sat_count,
                accuracy_m=gnss_acc_m
            )
        else:
            trust_score = 0.0

        self.mode_time_in_state += self.dt
        if self.mode_current_state == "GNSS_AIDED":
            if trust_score < self.mode_low_trust_threshold and self.mode_time_in_state >= self.mode_min_time_in_state:
                self.mode_current_state = "PURE_DEAD_RECKONING"
                self.mode_time_in_state = 0.0
        else:
            if trust_score > self.mode_high_trust_threshold and self.mode_time_in_state >= self.mode_min_time_in_state:
                self.mode_current_state = "GNSS_AIDED"
                self.mode_time_in_state = 0.0

        mode = self.mode_current_state

        if self.mode_current_state == "GNSS_AIDED" and is_gnss_available and gnss_pos_enu is not None:
            dynamic_sigma_pos = 5.0 / max(0.1, np.sqrt(trust_score))

            gnss_pos_passed, _, _ = self.ekf.update_gnss_position(
                p_gnss_enu=gnss_pos_enu,
                sigma_pos=dynamic_sigma_pos,
                alpha=0.01,
                timestamp=timestamp
            )
            if gnss_vel_enu is not None:
                dynamic_sigma_vel = 0.5 / max(0.2, trust_score)
                gnss_vel_passed, _, _ = self.ekf.update_gnss_velocity(
                    v_gnss_enu=gnss_vel_enu,
                    sigma_vel=dynamic_sigma_vel,
                    alpha=0.01,
                    timestamp=timestamp
                )
                speed_2d = np.linalg.norm(gnss_vel_enu[:2])
                if speed_2d >= 1.0:
                    if speed_2d >= 1.5:
                        sigma_heading = np.radians(3.0)
                    else:
                        sigma_heading = np.radians(3.0 + 7.0 * (1.5 - speed_2d) / 0.5)

                    cog_heading = float(np.arctan2(gnss_vel_enu[0], gnss_vel_enu[1]))
                    self.ekf.update_heading(
                        heading_rad=cog_heading,
                        sigma_heading=sigma_heading,
                        alpha=0.01,
                        timestamp=timestamp,
                        source="GNSS_HEADING"
                    )

        self.trajectory_pos.append(np.copy(self.ekf.p))
        self.trajectory_vel.append(np.copy(self.ekf.v))
        self.trajectory_cov_2d.append(np.copy(self.ekf.get_position_covariance_2d()))
        self.trajectory_euler.append(self.ekf.get_euler_angles_deg())
        self.trajectory_mode.append(mode)

        return {
            "pos": np.copy(self.ekf.p),
            "vel": np.copy(self.ekf.v),
            "cov_2d": np.copy(self.ekf.get_position_covariance_2d()),
            "euler_deg": self.ekf.get_euler_angles_deg(),
            "mode": mode,
            "vehicle_type": self.current_vehicle_type,
            "lean_angle_deg": float(np.degrees(self.current_lean_angle_rad)),
            "mag_disturbed": self.mag_gate.is_disturbed,
            "gnss_pos_passed": gnss_pos_passed,
            "gnss_vel_passed": gnss_vel_passed,
            "trust_score": float(trust_score)
        }

def eval_session(data_type, driver, session, k):
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
    fusion = CovScaledFusionEngine(dt=dt, default_vehicle_type=veh_type, k=k)

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

        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=m_raw,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage,
            timestamp=i*dt
        )
        results.append(res)

        if in_outage:
            outage_path.append([e_gt[i], n_gt[i]])

    pos_est = np.array([r["pos"] for r in results])
    pos_est = np.vstack([p0, pos_est])

    outage_path = np.array(outage_path)
    if len(outage_path) > 0:
        outage_dist = float(np.sum(np.sqrt(np.diff(outage_path[:, 0])**2 + np.diff(outage_path[:, 1])**2)))
        final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))
        drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0
    else:
        outage_dist = 0.0
        final_err = 0.0
        drift_pct = 0.0

    return drift_pct, final_err, outage_dist

if __name__ == "__main__":
    ks = [0.0, 50.0, 100.0, 250.0, 500.0, 1000.0]
    sessions = [
        ("Car S4", "car", "S (Driver A)", "S4"),
        ("TW Sess 2", "tw", "S (Driver A)", "session2"),
        ("TW Sess 1", "tw", "S (Driver A)", "session1"),
        ("Car S1", "car", "S (Driver A)", "S1"),
    ]

    print("=== Covariance-Scaled AI Speed Experiment (sigma_ai_eff = sigma_ai * (1 + k * yaw_var_rad2)) ===")
    for k in ks:
        print(f"\n--- k = {k} ---")
        for s_name, dtype, drv, sid in sessions:
            drift, err, dist = eval_session(dtype, drv, sid, k)
            print(f"  {s_name:12s}: Drift = {drift:10.2f}% (Err = {err:10.2f}m, Dist = {dist:6.1f}m)")
