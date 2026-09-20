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

class EdgeFusionEngine:
    def __init__(self, dt: float = 0.005, default_vehicle_type: str = "car"):
        self.dt = dt

        # FOG/Navigation-grade IMU tuning (substantially tighter thresholds than mobile MEMS)
        self.ekf = ErrorStateEKF(
            dt=dt,
            sigma_acc=0.01,           # FOG/Nav-grade has ultra-low noise
            sigma_gyro=0.001,
            sigma_acc_bias=1e-5,      # Stable bias, minimal drift
            sigma_gyro_bias=1e-6
        )

        self.constrained_ins = ConstrainedINS(dt=dt)
        self.lean_ekf = LeanAngleEKF(dt=dt)
        self.current_vehicle_type = default_vehicle_type

    def initialize_state(
        self,
        p0_enu: np.ndarray,
        v0_enu: np.ndarray,
        heading0_deg: float,
    ):
        psi = np.radians(heading0_deg)
        R_veh_to_nav = np.array([
            [np.cos(psi), np.sin(psi), 0.0],
            [-np.sin(psi), np.cos(psi), 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        from engine.strapdown import dcm_to_quat
        q0 = dcm_to_quat(R_veh_to_nav)

        self.ekf.set_initial_state(
            p0=p0_enu,
            v0=v0_enu,
            q0=q0,
            b_a0=np.zeros(3),
            b_g0=np.zeros(3)
        )

    def step(
        self,
        acc_raw: np.ndarray,
        gyro_raw: np.ndarray,
        gnss_pos_enu: Optional[np.ndarray] = None,
        gnss_vel_enu: Optional[np.ndarray] = None,
        timestamp: float = 0.0
    ) -> Dict:
        # FOG/Edge path: Simplified processing
        # 1. State Propagation (Classical backbone)
        self.ekf.predict(acc_raw=acc_raw, gyro_raw=gyro_raw, dt=self.dt, Q_scale=1.0)

        # 2. Constraints (NHC/ZUPT)
        v_veh = self.ekf.quat_to_rot(self.ekf.q).T @ self.ekf.v
        is_stopped = self.constrained_ins._is_stopped(acc_raw, gyro_raw, v_veh)

        if is_stopped:
            self.ekf.update_zupt(sigma_zupt=0.01, alpha=0.01, timestamp=timestamp)
        else:
            self.ekf.update_nhc(
                vehicle_type=self.current_vehicle_type,
                lean_angle_rad=0.0, # Lean-compensation FOG path (optional), simplified here
                sigma_nhc_x=0.05,
                sigma_nhc_z=0.05,
                alpha=0.01,
                timestamp=timestamp
            )

        # 3. GNSS updates (NIS gated update)
        if gnss_pos_enu is not None:
             self.ekf.update_gnss_position(
				p_gnss_enu=gnss_pos_enu,
				sigma_pos=1.0,
				alpha=0.01,
				timestamp=timestamp
			)

        if gnss_vel_enu is not None:
             self.ekf.update_gnss_velocity(
				v_gnss_enu=gnss_vel_enu,
				sigma_vel=0.1,
				alpha=0.01,
				timestamp=timestamp
			)

        return {
            "pos": np.copy(self.ekf.p),
            "vel": np.copy(self.ekf.v),
            "cov_2d": np.copy(self.ekf.get_position_covariance_2d()),
            "euler_deg": self.ekf.get_euler_angles_deg()
        }
