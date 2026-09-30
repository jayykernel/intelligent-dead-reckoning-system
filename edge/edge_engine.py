"""
edge/edge_engine.py

Edge-deployable fusion engine wrapper for FOG-grade / Navigation-grade IMUs.
Uses shared classical strapdown mechanization + ES-EKF backbone, but bypasses MEMS-specific
AI correction/noise models (N7) and relies on fixed-frame assumptions.

Output interface: position, velocity, covariance, heading.
"""

import numpy as np
from typing import Dict, Optional

from engine.fusion.ekf import ErrorStateEKF
from engine.nhc_zupt.constrained_ins import ConstrainedINS
from engine.nhc_zupt.lean_ekf import LeanAngleEKF
from engine.calibration.calibrator import CalibrationEngine

class EdgeFusionEngine:
    def __init__(self, dt: float = 0.005, default_vehicle_type: str = "car"):
        self.dt = dt
        self.ai_speed_scale = 1.0
        self.calibrator = CalibrationEngine()  # Per-unit calibration (N8)

        # FOG/Navigation-grade IMU tuning (substantially tighter thresholds than mobile MEMS)
        self.ekf = ErrorStateEKF(
            dt=dt,
            sigma_acc=0.001,          # FOG/Nav-grade has ultra-low noise
            sigma_gyro=0.0001,
            sigma_acc_bias=1e-6,      # Stable bias, minimal drift
            sigma_gyro_bias=1e-7
        )
        self.ekf.gyro_mag_scale = 0.0 # pure stable integration

        self.constrained_ins = ConstrainedINS(dt=dt)
        self.lean_ekf = LeanAngleEKF(dt=dt)
        self.current_vehicle_type = default_vehicle_type

    def initialize_state(
        self,
        p0_enu: np.ndarray,
        v0_enu: np.ndarray,
        heading0_deg: float,
        acc_raw: np.ndarray = None,
        gyro_raw: np.ndarray = None,
        speed_samples: np.ndarray = None,
        dt: float = 0.005
    ):
        """
        Initialize the engine state and run per-unit calibration (N8).

        For FOG/Nav-grade units, calibration covers unit-to-unit variance assuming rigid vehicle mounting.
        """
        # Run calibration if initial IMU samples provided
        if acc_raw is not None and gyro_raw is not None and speed_samples is not None:
            self.calibrator.calibrate_from_session(acc_raw, gyro_raw, speed_samples, dt=dt)

        psi = np.radians(heading0_deg)
        R_veh_to_nav = np.array([
            [np.cos(psi), np.sin(psi), 0.0],
            [-np.sin(psi), np.cos(psi), 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        from engine.strapdown import dcm_to_quat
        q0 = dcm_to_quat(R_veh_to_nav)

        # Use calibrated biases for initial state
        b_a0 = self.calibrator.accel_bias if self.calibrator.is_calibrated else np.zeros(3)
        b_g0 = self.calibrator.gyro_bias if self.calibrator.is_calibrated else np.zeros(3)

        self.ekf.set_initial_state(
            p0=p0_enu,
            v0=v0_enu,
            q0=q0,
            b_a0=b_a0,
            b_g0=b_g0
        )
        self.ekf.P[0:3, 0:3] = np.eye(3) * 0.01**2
        self.ekf.P[3:6, 3:6] = np.eye(3) * 0.01**2
        self.ekf.P[6:9, 6:9] = np.eye(3) * np.radians(0.01)**2
        self.ekf.P[9:12, 9:12] = np.eye(3) * 1e-6**2
        self.ekf.P[12:15, 12:15] = np.eye(3) * 1e-7**2

    def step(
        self,
        acc_raw: np.ndarray,
        gyro_raw: np.ndarray,
        gnss_pos_enu: Optional[np.ndarray] = None,
        gnss_vel_enu: Optional[np.ndarray] = None,
        timestamp: float = 0.0,
        ai_speed: Optional[float] = None,
        lean_angle_rad: float = 0.0
    ) -> Dict:
        # FOG/Edge path: Simplified processing
        # 1. Apply calibration to raw IMU inputs (N8)
        acc_veh, gyro_veh = self.calibrator.apply(acc_raw, gyro_raw)

        # 2. State Propagation (Classical backbone)
        self.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=self.dt, Q_scale=1.0)

        # 3. Constraints (NHC/ZUPT) using calibrated vehicle-frame IMU
        v_veh = self.ekf.quat_to_rot(self.ekf.q).T @ self.ekf.v
        is_stopped = self.constrained_ins._is_stopped(acc_veh, gyro_veh, v_veh)

        if is_stopped:
            self.ekf.update_zupt(sigma_zupt=0.01, alpha=0.01, timestamp=timestamp)
            self.ekf.update_zero_angular_rate(
                gyro_veh=gyro_veh,
                sigma_gyro_bias=0.001,
                alpha=0.01,
                timestamp=timestamp
            )
        else:
            self.ekf.update_nhc(
                vehicle_type=self.current_vehicle_type,
                lean_angle_rad=lean_angle_rad,
                sigma_nhc_x=0.01,
                sigma_nhc_z=0.01,
                alpha=0.01,
                timestamp=timestamp
            )

        if ai_speed is not None and self.current_vehicle_type != "two_wheeler":
            # Learn speed scale if gnss vel available
            if gnss_vel_enu is not None:
                gnss_speed = np.linalg.norm(gnss_vel_enu[:2])
                if gnss_speed > 3.0 and ai_speed > 3.0:
                    ratio = gnss_speed / ai_speed
                    self.ai_speed_scale = 0.995 * self.ai_speed_scale + 0.005 * np.clip(ratio, 0.7, 1.3)

            # Inject speed
            scaled_speed = ai_speed * self.ai_speed_scale
            self.ekf.update_ai_forward_speed(
                speed_fwd=scaled_speed,
                sigma_speed=0.5,
                alpha=0.01,
                timestamp=timestamp
            )

        # 4. GNSS updates (NIS gated update)
        if gnss_pos_enu is not None:
            self.ekf.update_gnss_position(
                p_gnss_enu=gnss_pos_enu,
                sigma_pos=0.1,
                alpha=0.01,
                timestamp=timestamp
            )

        if gnss_vel_enu is not None:
            self.ekf.update_gnss_velocity(
                v_gnss_enu=gnss_vel_enu,
                sigma_vel=0.05,
                alpha=0.01,
                timestamp=timestamp
            )

        return {
            "pos": np.copy(self.ekf.p),
            "vel": np.copy(self.ekf.v),
            "cov_2d": np.copy(self.ekf.get_position_covariance_2d()),
            "euler_deg": self.ekf.get_euler_angles_deg()
        }