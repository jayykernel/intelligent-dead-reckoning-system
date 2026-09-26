"""
engine/fusion/fusion_engine.py

Unified GNSS+INS Fusion Engine.

Integrates:
- 15-State Error-State Extended Kalman Filter (ES-EKF)
- Chi-squared (NIS) Innovation Gating (N3)
- Magnetometer Disturbance Gating (N5)
- AI Speed & Vibration Correction Module (N7, MEMS path)
- Vehicle-Type Classifier (N6) with 5-step Hysteresis
- Lean-Angle EKF & Lean-Compensated NHC / ZUPT (N1)
- Phone-to-Vehicle Calibration (N8)
"""

import os
import numpy as np
from typing import Dict, Tuple, Optional, List

from engine.fusion.ekf import ErrorStateEKF
from engine.fusion.mag_gate import MagnetometerGate
from engine.fusion.ai_corrector import AICorrectionModule
from engine.outage_prediction.outage_predictor import OutagePredictor
from engine.ai_filters.vehicle_classifier import VehicleClassifier
from engine.nhc_zupt.lean_ekf import LeanAngleEKF
from engine.nhc_zupt.constrained_ins import ConstrainedINS
from engine.calibration.calibrator import CalibrationEngine
from engine.strapdown import dcm_to_quat


class GNSSINSFusionEngine:
    def __init__(
        self,
        dt: float = 0.1,
        speed_model_path: str = "training/models/speed_filter.tflite",
        class_model_path: str = "training/models/vehicle_classifier.tflite",
        enable_ai: bool = True,
        default_vehicle_type: str = "car"
    ):
        self.dt = dt
        self.enable_ai = enable_ai

        # Core EKF with smartphone tuning
        self.ekf = ErrorStateEKF(
            dt=dt,
            sigma_acc=5.0,           # Phone IMU has high noise/vibration (requires 5.0+ to track AI speed during bumps)
            sigma_gyro=0.1,          # Gyro is also noisy
            sigma_acc_bias=0.05,     # Aggressive bias tracking to handle mount shifts/pitch
            sigma_gyro_bias=0.01
        )

        # Calibration
        self.calib = CalibrationEngine()

        # Phase 5 Modules
        self.classifier: Optional[VehicleClassifier] = None
        if os.path.exists(class_model_path) and self.enable_ai:
            try:
                self.classifier = VehicleClassifier(model_path=class_model_path)
            except Exception as e:
                print(f"[FusionEngine] Classifier init warning: {e}")
                self.classifier = None

        self.lean_ekf = LeanAngleEKF(dt=dt)
        self.constrained_ins = ConstrainedINS(dt=dt)

        # Novelty Modules
        self.mag_gate = MagnetometerGate()
        self.ai_corrector = AICorrectionModule(
            model_path=speed_model_path,
            enable_tflite=enable_ai
        )
        self.outage_predictor = OutagePredictor()

        # Buffers & States
        self.current_vehicle_type = default_vehicle_type
        self.current_lean_angle_rad = 0.0
        self.classifier_buffer: List[np.ndarray] = []
        self.classifier_window_size = 20

        # Seamless Mode Handler (Phase 9) state machine
        self.mode_low_trust_threshold = 0.2   # below this, consider transitioning to pure INS
        self.mode_high_trust_threshold = 0.8  # above this, consider transitioning to GNSS aided
        self.mode_min_time_in_state = 1.0     # minimum seconds to stay in a state before allowing a transition back
        self.mode_current_state = "GNSS_AIDED"  # start in GNSS aided assuming we have good signal initially
        self.mode_time_in_state = 0.0

        # State histories for logging
        self.trajectory_pos = []
        self.trajectory_vel = []
        self.trajectory_cov_2d = []
        self.trajectory_euler = []
        self.trajectory_mode = []

    def initialize_state(
        self,
        p0_enu: np.ndarray,
        v0_enu: np.ndarray,
        heading0_deg: float,
        acc0_raw: np.ndarray,
        R_phone_to_veh: Optional[np.ndarray] = None,
        gyro_bias: Optional[np.ndarray] = None,
        accel_bias: Optional[np.ndarray] = None
    ):
        """
        Initialize the EKF nominal state from initial GPS position, velocity, and heading.
        """
        if R_phone_to_veh is not None:
            self.calib.R_phone_to_veh = np.copy(R_phone_to_veh)
            self.calib.is_calibrated = True
        if gyro_bias is not None:
            self.calib.gyro_bias = np.copy(gyro_bias)
        if accel_bias is not None:
            self.calib.accel_bias = np.copy(accel_bias)

        # Initial orientation quaternion from level gravity and heading
        # Vehicle frame: X Right, Y Forward, Z Up
        psi = np.radians(heading0_deg)
        # R_veh_to_nav (Body to Nav) to align Forward (Y) with psi (clockwise from North, ENU):
        # Forward (Y) = [sin(psi), cos(psi), 0]
        # Right (X) = [cos(psi), -sin(psi), 0] (since Y x X = -Z, wait right-handed X Right, Y Forward, Z Up: X x Y = Z)
        # XRight x YForward = ZUp! Correct!
        # [cos(psi), -sin(psi), 0] x [sin(psi), cos(psi), 0] = [0, 0, cos^2+sin^2] = [0, 0, 1]! Correct!
        R_veh_to_nav = np.array([
            [np.cos(psi), np.sin(psi), 0.0],
            [-np.sin(psi), np.cos(psi), 0.0], # Wait: X-right, Y-forward, Z-up: X=[1,0,0], Y=[0,1,0]. Rotation Z->East(psi)!
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        # R_veh_to_nav column 0 is Vehicle X (Right), Column 1 is Vehicle Y (Forward).
        # We want Vehicle Forward (Y) to be [sin(psi), cos(psi), 0] (Nav North=Y in ENU).
        # Vehicle Forward = R * [0, 1, 0]^T = R[:, 1]!
        # So column 1 is [sin(psi), cos(psi), 0]^T!
        # And Vehicle X = [cos(psi), -sin(psi), 0]^T!
        R_veh_to_nav = np.array([
            [np.cos(psi), np.sin(psi), 0.0],
            [-np.sin(psi), np.cos(psi), 0.0], # Wait, Row 1 is North? [cos, sin, 0] * [0,1,0]^T is sin(psi)!
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        # R * [0,1,0]^T = [col0, col1, col2] * [0,1,0]^T = col1.
        # Column 1 = [sin(psi), cos(psi), 0]^T!
        # Column 0 = must be [cos(psi), -sin(psi), 0]^T!
        R_veh_to_nav = np.array([
            [np.cos(psi), np.sin(psi), 0.0],
            [-np.sin(psi), np.cos(psi), 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        # This Column 1 is [sin(psi), cos(psi), 0], Column 0 is [cos(psi), -sin(psi), 0]!
        # X-Right = [cos, -sin, 0], Y-Forward = [sin, cos, 0].
        # X x Y = Z.
        # [cos, -sin, 0] x [sin, cos, 0] = [0, 0, cos^2 + sin^2] = [0, 0, 1]! Yes.
        # This rotation matrix is correct!
        q0 = dcm_to_quat(R_veh_to_nav)

        self.ekf.set_initial_state(
            p0=p0_enu,
            v0=v0_enu,
            q0=q0,
            b_a0=np.zeros(3),  # Initial biases are 0 in EKF because calib.apply() pre-subtracts them
            b_g0=np.zeros(3)
        )

        self.trajectory_pos.append(np.copy(p0_enu))
        self.trajectory_vel.append(np.copy(v0_enu))
        self.trajectory_cov_2d.append(np.copy(self.ekf.get_position_covariance_2d()))
        self.trajectory_euler.append(self.ekf.get_euler_angles_deg())
        self.trajectory_mode.append("GNSS_AIDED")

    def step(
        self,
        acc_raw: np.ndarray,
        gyro_raw: np.ndarray,
        mag_raw: Optional[np.ndarray] = None,
        gnss_pos_enu: Optional[np.ndarray] = None,
        gnss_vel_enu: Optional[np.ndarray] = None,
        is_gnss_available: bool = True,
        timestamp: float = 0.0,
        gnss_acc_m: Optional[float] = None,
        gnss_sat_count: Optional[int] = None,
        gnss_avg_cn0: Optional[float] = None
    ) -> Dict:
        """
        Execute one fusion step at dt (10 Hz).

        acc_raw: (3,) Raw phone accelerometer (m/s^2)
        gyro_raw: (3,) Raw phone gyroscope (rad/s)
        mag_raw: (3,) Optional raw phone magnetometer (uT)
        gnss_pos_enu: (3,) Optional GNSS position fix in ENU (m)
        gnss_vel_enu: (3,) Optional GNSS velocity fix in ENU (m/s)
        is_gnss_available: Boolean flag indicating if GNSS is currently active
        timestamp: Current timestamp (s)

        Returns:
        Dictionary containing estimated state, uncertainties, mode, and gating flags.
        """
        # 1. Apply calibration to convert Phone IMU -> Vehicle IMU
        acc_veh, gyro_veh = self.calib.apply(acc_raw, gyro_raw)

        # 2. Update Vehicle-Type Classifier (N6)
        if self.classifier is not None:
            feat_sample = np.hstack([acc_veh, gyro_veh])
            self.classifier_buffer.append(feat_sample)
            if len(self.classifier_buffer) > self.classifier_window_size:
                self.classifier_buffer.pop(0)

            if len(self.classifier_buffer) == self.classifier_window_size:
                acc_win = np.array([f[:3] for f in self.classifier_buffer])
                gyro_win = np.array([f[3:] for f in self.classifier_buffer])
                self.current_vehicle_type = self.classifier.predict_window(acc_win, gyro_win)

        # 3. AI Correction Module (N7) - Process vibration & predict speed
        ai_speed, sigma_ai, q_scale = self.ai_corrector.process_imu_sample(
            acc_raw=acc_raw,
            gyro_raw=gyro_raw
        )

        # 4. EKF State Propagation
        self.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=self.dt, Q_scale=q_scale)

        # 5. Update Lean-Angle EKF (N1) for Two-Wheelers
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

        # 6. Apply Continuous Non-Holonomic Constraints (NHC) & ZUPT via EKF Measurement Updates
        # Active continuously as an aiding source regardless of GNSS availability
        is_stopped = self.constrained_ins._is_stopped(acc_veh, gyro_veh, v_veh, ai_speed=ai_speed)
        if is_stopped:
            # Stationary vehicle: 3D zero velocity update
            self.ekf.update_zupt(
                sigma_zupt=0.05,
                alpha=0.01,
                timestamp=timestamp
            )
            # Zero Angular Rate Update (ZARU) to continuously observe and re-zero gyro bias
            self.ekf.update_zero_angular_rate(
                gyro_veh=gyro_veh,
                sigma_gyro_bias=0.02,
                alpha=0.01,
                timestamp=timestamp
            )
        else:
            # Moving vehicle: continuous NHC (lateral and vertical constraints, lean-compensated for two-wheelers)
            self.ekf.update_nhc(
                vehicle_type=self.current_vehicle_type,
                lean_angle_rad=self.current_lean_angle_rad,
                sigma_nhc_x=0.2,
                sigma_nhc_z=0.2,
                alpha=0.01,
                timestamp=timestamp
            )

        # Apply AI Forward Speed update continuously as an aiding measurement
        if ai_speed is not None:
            self.ekf.update_ai_forward_speed(
                speed_fwd=ai_speed,
                sigma_speed=sigma_ai,
                alpha=0.01,
                timestamp=timestamp
            )

        # 7. Magnetometer Disturbance Gating (N5)
        mag_used = False
        if mag_raw is not None:
            # Rotate mag to vehicle frame
            mag_veh = mag_raw @ self.calib.R_phone_to_veh.T
            is_clean, mag_yaw, mag_info = self.mag_gate.process_measurement(mag_veh, R_veh_to_nav)
            if is_clean and mag_yaw is not None:
                # Apply 1-DOF NIS gated heading update whenever magnetometer is clean
                passed, _, _ = self.ekf.update_heading(
                    heading_rad=mag_yaw,
                    sigma_heading=np.radians(8.0),
                    alpha=0.01,
                    timestamp=timestamp,
                    source="MAG_HEADING"
                )
                mag_used = passed

        # 8. GNSS Fix Updates with Predictive Trust Scaling & NIS Gating
        gnss_pos_passed = False
        gnss_vel_passed = False

        # Calculate early trust signal using raw GNSS measurements
        if is_gnss_available:
            trust_score = self.outage_predictor.update(
                avg_cn0=gnss_avg_cn0,
                sat_count=gnss_sat_count,
                accuracy_m=gnss_acc_m
            )
        else:
            # When GNSS is not available, trust it zero.
            trust_score = 0.0

        # Update the seamless mode handler state machine (Phase 9)
        self.mode_time_in_state += self.dt
        if self.mode_current_state == "GNSS_AIDED":
            if trust_score < self.mode_low_trust_threshold and self.mode_time_in_state >= self.mode_min_time_in_state:
                # Transition to pure INS
                self.mode_current_state = "PURE_DEAD_RECKONING"
                self.mode_time_in_state = 0.0
        else:  # current state is PURE_DEAD_RECKONING
            if trust_score > self.mode_high_trust_threshold and self.mode_time_in_state >= self.mode_min_time_in_state:
                # Transition to GNSS aided
                self.mode_current_state = "GNSS_AIDED"
                self.mode_time_in_state = 0.0

        # The mode is now determined by the state machine
        mode = self.mode_current_state

        if self.mode_current_state == "GNSS_AIDED" and is_gnss_available and gnss_pos_enu is not None:
            # Dynamically scale the measurement uncertainty based on trust
            # lower trust -> higher uncertainty -> less weight on GNSS
            dynamic_sigma_pos = 5.0 / max(0.1, np.sqrt(trust_score))

            gnss_pos_passed, _, _ = self.ekf.update_gnss_position(
                p_gnss_enu=gnss_pos_enu,
                sigma_pos=dynamic_sigma_pos,
                alpha=0.01,
                timestamp=timestamp
            )
            if gnss_vel_enu is not None:
                # Also scale velocity uncertainty slightly
                dynamic_sigma_vel = 0.5 / max(0.2, trust_score)
                gnss_vel_passed, _, _ = self.ekf.update_gnss_velocity(
                    v_gnss_enu=gnss_vel_enu,
                    sigma_vel=dynamic_sigma_vel,
                    alpha=0.01,
                    timestamp=timestamp
                )
                # GNSS Course-Over-Ground (COG) provides absolute heading
                speed_2d = np.linalg.norm(gnss_vel_enu[:2])

                # Minimum speed gate for COG heading: skip completely if below 1.0 m/s
                if speed_2d >= 1.0:
                    if speed_2d >= 1.5:
                        sigma_heading = np.radians(3.0)  # Tight heading estimate at higher speeds
                    else:
                        sigma_heading = np.radians(3.0 + 7.0 * (1.5 - speed_2d) / 0.5)  # Looser at low speeds

                    cog_heading = float(np.arctan2(gnss_vel_enu[0], gnss_vel_enu[1]))
                    self.ekf.update_heading(
                        heading_rad=cog_heading,
                        sigma_heading=sigma_heading,
                        alpha=0.01,
                        timestamp=timestamp,
                        source="GNSS_HEADING"
                    )

        # Record histories
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
