"""
engine/fusion/ekf.py

15-State Error-State Extended Kalman Filter (ES-EKF) for GNSS+INS Fusion.

Nominal States (16-dim representation):
- Position: p_nav = [p_E, p_N, p_U] (m) in local ENU frame
- Velocity: v_nav = [v_E, v_N, v_U] (m/s) in local ENU frame
- Orientation: quat = [qw, qx, qy, qz] (Vehicle/Body to Navigation frame)
- Accel Bias: b_a = [b_ax, b_ay, b_az] (m/s^2) in Vehicle frame
- Gyro Bias: b_g = [b_gx, b_gy, b_gz] (rad/s) in Vehicle frame

Error States (15-dim):
- delta_x = [delta_p (3), delta_v (3), delta_theta (3), delta_b_a (3), delta_b_g (3)]

Features:
- Numerically stable Joseph-form covariance update
- Chi-squared (NIS) innovation gating
- Support for 3D Position, 3D Velocity, and 1D Heading/Speed updates
"""

import numpy as np
from typing import Dict, Tuple, Optional, List
from scipy.stats import chi2


class ErrorStateEKF:
    def __init__(
        self,
        dt: float = 0.1,
        sigma_acc: float = 0.2,       # Accelerometer noise (m/s^2)
        sigma_gyro: float = 0.02,     # Gyroscope noise (rad/s)
        sigma_acc_bias: float = 0.001,# Accel bias random walk (m/s^2 / sqrt(s))
        sigma_gyro_bias: float = 0.0001,# Gyro bias random walk (rad/s / sqrt(s))
    ):
        self.dt = dt
        self.sigma_acc = sigma_acc
        self.sigma_gyro = sigma_gyro
        self.sigma_acc_bias = sigma_acc_bias
        self.sigma_gyro_bias = sigma_gyro_bias

        # Nominal states
        self.p = np.zeros(3)  # Position ENU
        self.v = np.zeros(3)  # Velocity ENU
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [qw, qx, qy, qz] (veh -> nav)
        self.b_a = np.zeros(3)  # Accel bias
        self.b_g = np.zeros(3)  # Gyro bias

        # 15x15 Error state covariance
        self.P = np.eye(15)
        # Initial uncertainties
        self.P[0:3, 0:3] = np.eye(3) * 5.0**2     # Position initial std 5m
        self.P[3:6, 3:6] = np.eye(3) * 1.0**2     # Velocity initial std 1 m/s
        self.P[6:9, 6:9] = np.eye(3) * np.radians(5.0)**2  # Attitude initial std 5 deg
        self.P[9:12, 9:12] = np.eye(3) * 0.1**2   # Accel bias initial std 0.1 m/s^2
        self.P[12:15, 12:15] = np.eye(3) * 0.01**2 # Gyro bias initial std 0.01 rad/s

        # Gravity in navigation frame (ENU: Z is Up)
        self.g_nav = np.array([0.0, 0.0, -9.80665])

        # Logging for NIS gating
        self.nis_history: List[Dict] = []

    def set_initial_state(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        q0: np.ndarray,
        b_a0: Optional[np.ndarray] = None,
        b_g0: Optional[np.ndarray] = None,
    ):
        """Set initial nominal state and normalize quaternion."""
        self.p = np.copy(p0)
        self.v = np.copy(v0)
        self.q = np.copy(q0) / np.linalg.norm(q0)
        if b_a0 is not None:
            self.b_a = np.copy(b_a0)
        if b_g0 is not None:
            self.b_g = np.copy(b_g0)

    @staticmethod
    def quat_to_rot(q: np.ndarray) -> np.ndarray:
        """Convert unit quaternion [qw, qx, qy, qz] to 3x3 rotation matrix R (veh to nav)."""
        qw, qx, qy, qz = q
        return np.array([
            [1.0 - 2.0*(qy**2 + qz**2), 2.0*(qx*qy - qw*qz), 2.0*(qx*qz + qw*qy)],
            [2.0*(qx*qy + qw*qz), 1.0 - 2.0*(qx**2 + qz**2), 2.0*(qy*qz - qw*qx)],
            [2.0*(qx*qz - qw*qy), 2.0*(qy*qz + qw*qx), 1.0 - 2.0*(qx**2 + qy**2)]
        ])

    @staticmethod
    def skew_symmetric(v: np.ndarray) -> np.ndarray:
        """Create 3x3 skew symmetric cross-product matrix [v]x."""
        return np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0]
        ])

    def predict(self, acc_raw: np.ndarray, gyro_raw: np.ndarray, dt: Optional[float] = None, Q_scale: float = 1.0):
        """
        Nominal state integration and error covariance propagation.
        acc_raw: (3,) Specific force in vehicle frame
        gyro_raw: (3,) Angular rate in vehicle frame
        """
        if dt is None:
            dt = self.dt

        # 1. Bias-corrected IMU measurements
        acc_corr = acc_raw - self.b_a
        gyro_corr = gyro_raw - self.b_g

        # 2. Update nominal orientation
        rot_vec = gyro_corr * dt
        angle = np.linalg.norm(rot_vec)
        if angle < 1e-12:
            dq = np.array([1.0 - angle**2 / 8.0, rot_vec[0] / 2.0, rot_vec[1] / 2.0, rot_vec[2] / 2.0])
        else:
            axis = rot_vec / angle
            half_angle = angle / 2.0
            sin_half = np.sin(half_angle)
            dq = np.array([np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half])

        # Quaternion multiplication: q_new = q * dq (body-frame rotation)
        qw, qx, qy, qz = self.q
        dw, dx, dy, dz = dq
        self.q = np.array([
            qw*dw - qx*dx - qy*dy - qz*dz,
            qw*dx + qx*dw + qy*dz - qz*dy,
            qw*dy - qx*dz + qy*dw + qz*dx,
            qw*dz + qx*dy - qy*dx + qz*dw
        ])
        self.q = self.q / np.linalg.norm(self.q)

        # 3. Specific force in Nav frame
        R = self.quat_to_rot(self.q)
        f_nav = R @ acc_corr

        # Acceleration in Nav frame
        a_nav = f_nav + self.g_nav

        # 4. Integrate Position and Velocity
        self.p += self.v * dt + 0.5 * a_nav * dt**2
        self.v += a_nav * dt

        # 5. Continuous-to-Discrete Error State Transition Matrix F
        # delta_x = [delta_p (0..2), delta_v (3..5), delta_theta (6..8), delta_b_a (9..11), delta_b_g (12..14)]
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[0:3, 6:9] = -0.5 * R @ self.skew_symmetric(acc_corr) * dt**2
        F[0:3, 9:12] = -0.5 * R * dt**2  # Correct negative sign for accel bias coupling

        F[3:6, 6:9] = -R @ self.skew_symmetric(acc_corr) * dt
        F[3:6, 9:12] = -R * dt           # Correct negative sign for accel bias coupling

        # F[6:9, 6:9] = I - skew(gyro_corr) * dt
        F[6:9, 6:9] = np.eye(3) - self.skew_symmetric(gyro_corr) * dt
        # F[6:9, 12:15] = -I * dt (Bias is in Body frame)
        F[6:9, 12:15] = -np.eye(3) * dt

        # 6. Process Noise Covariance Q
        Q = np.zeros((15, 15))
        # Position noise from acc integration
        Q[0:3, 0:3] = np.eye(3) * (0.5 * (self.sigma_acc * Q_scale) * dt**2)**2
        # Velocity noise from acc
        Q[3:6, 3:6] = np.eye(3) * ((self.sigma_acc * Q_scale) * dt)**2

        # Attitude noise from gyro: adaptive scaling during fast maneuvers
        # High angular rates increase integration errors and scale factor uncertainties
        # Scale proportionally to gyro magnitude to capture gyro integration error ~= gyro_rate * dt
        gyro_mag = np.linalg.norm(gyro_corr)
        adaptive_sigma_gyro = self.sigma_gyro * Q_scale + 2.0 * gyro_mag  # Add 2x rate magnitude as uncertainty
        Q[6:9, 6:9] = np.eye(3) * (adaptive_sigma_gyro * dt)**2

        # Bias random walks
        Q[9:12, 9:12] = np.eye(3) * (self.sigma_acc_bias**2 * dt)
        Q[12:15, 12:15] = np.eye(3) * (self.sigma_gyro_bias**2 * dt)

        # 7. Covariance propagation
        self.P = F @ self.P @ F.T + Q
        # Enforce symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update(
        self,
        z: np.ndarray,
        h_x: np.ndarray,
        H: np.ndarray,
        R_cov: np.ndarray,
        update_type: str = "GNSS_POS",
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """
        Generic measurement update with Chi-squared NIS innovation gating.

        z: (M,) Actual measurement vector
        h_x: (M,) Expected measurement vector from nominal state
        H: (M, 15) Measurement Jacobian w.r.t error state delta_x
        R_cov: (M, M) Measurement noise covariance matrix
        update_type: Label for logging (e.g. "GNSS_POS", "GNSS_VEL", "MAG_HEADING", "AI_SPEED")
        alpha: Significance level for Chi-squared test (default 0.01 => 99% acceptance interval)

        Returns:
        (passed_gate, nis_val, chi2_threshold)
        """
        # 1. Innovation vector
        y = z - h_x
        if update_type in ["MAG_HEADING", "GNSS_HEADING", "MAP_HEADING"]:
            # Wrap angular innovation to [-pi, pi]
            y[0] = (y[0] + np.pi) % (2 * np.pi) - np.pi

        # 2. Innovation Covariance
        S = H @ self.P @ H.T + R_cov
        S = 0.5 * (S + S.T)

        # 3. Normalized Innovation Squared (NIS)
        try:
            S_inv = np.linalg.inv(S)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
            nis = float(y.T @ S_inv @ y)

        dof = len(z)
        chi2_thresh = float(chi2.ppf(1.0 - alpha, df=dof))
        passed = nis <= chi2_thresh

        # Log NIS event
        self.nis_history.append({
            "timestamp": timestamp,
            "type": update_type,
            "dof": dof,
            "nis": nis,
            "threshold": chi2_thresh,
            "passed": passed,
            "innovation": y.tolist()
        })

        force_accept = update_type in ["GNSS_POS", "GNSS_VEL", "GNSS_HEADING", "MAP_POS", "ZUPT", "NHC", "ZARU", "AI_SPEED"]
        if not passed and not force_accept:
            # Gate rejects the inconsistent measurement
            return False, nis, chi2_thresh

        # 5. Kalman Gain
        K = self.P @ H.T @ S_inv  # (15, M)

        # 6. Error state correction
        delta_x = K @ y  # (15,)

        # 7. Joseph form covariance update: P = (I - K H) P (I - K H)^T + K R K^T
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_cov @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        # 8. Inject error states into nominal state
        self.inject_error_state(delta_x)

        return True, nis, chi2_thresh

    def inject_error_state(self, delta_x: np.ndarray):
        """Inject 15-state error vector into nominal state and reset error state."""
        delta_p = delta_x[0:3]
        delta_v = delta_x[3:6]
        delta_theta = delta_x[6:9]
        delta_ba = delta_x[9:12]
        delta_bg = delta_x[12:15]

        # 1. Correct position and velocity
        self.p += delta_p
        self.v += delta_v

        # 2. Correct attitude quaternion: q_new = q * dq(delta_theta)
        d_angle = np.linalg.norm(delta_theta)
        if d_angle < 1e-12:
            dq = np.array([1.0, 0.5 * delta_theta[0], 0.5 * delta_theta[1], 0.5 * delta_theta[2]])
        else:
            axis = delta_theta / d_angle
            half = d_angle / 2.0
            dq = np.array([np.cos(half), axis[0]*np.sin(half), axis[1]*np.sin(half), axis[2]*np.sin(half)])

        # q_new = q * dq (body-frame correction)
        qw, qx, qy, qz = self.q
        dw, dx, dy, dz = dq
        self.q = np.array([
            qw*dw - qx*dx - qy*dy - qz*dz,
            qw*dx + qx*dw + qy*dz - qz*dy,
            qw*dy - qx*dz + qy*dw + qz*dx,
            qw*dz + qx*dy - qy*dx + qz*dw
        ])
        self.q = self.q / np.linalg.norm(self.q)

        # 3. Correct biases
        self.b_a += delta_ba
        self.b_g += delta_bg

    def update_gnss_position(
        self,
        p_gnss_enu: np.ndarray,
        sigma_pos: float = 3.0,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """Update with 3D GNSS Position measurement in ENU."""
        z = p_gnss_enu  # (3,)
        h_x = self.p    # (3,)
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        R_cov = np.eye(3) * (sigma_pos**2)

        return self.update(z, h_x, H, R_cov, update_type="GNSS_POS", alpha=alpha, timestamp=timestamp)

    def update_gnss_velocity(
        self,
        v_gnss_enu: np.ndarray,
        sigma_vel: float = 0.5,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """Update with 3D GNSS Velocity measurement in ENU."""
        z = v_gnss_enu  # (3,)
        h_x = self.v    # (3,)
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        R_cov = np.eye(3) * (sigma_vel**2)

        return self.update(z, h_x, H, R_cov, update_type="GNSS_VEL", alpha=alpha, timestamp=timestamp)

    def update_heading(
        self,
        heading_rad: float,
        sigma_heading: float = np.radians(5.0),
        alpha: float = 0.01,
        timestamp: float = 0.0,
        source: str = "MAG_HEADING"
    ) -> Tuple[bool, float, float]:
        """
        Update with absolute Heading measurement (geographic heading: 0 = North, +pi/2 = East, in rad).
        In ENU Navigation frame:
        Vehicle Y (forward) column in R: R[0, 1] = East, R[1, 1] = North.
        Geographic heading psi = atan2(R[0, 1], R[1, 1]) = atan2(East, North).
        """
        R = self.quat_to_rot(self.q)
        current_yaw = float(np.arctan2(R[0, 1], R[1, 1]))

        z = np.array([heading_rad])
        h_x = np.array([current_yaw])

        # Jacobian w.r.t delta_theta_z (body frame):
        # R_new[:, 1] = R[:, 1] - R[:, 0] * delta_theta_z => psi_new = psi - delta_theta_z
        # So d(psi)/d(delta_theta_z) = -1.0
        H = np.zeros((1, 15))
        H[0, 8] = -1.0  # delta_theta_z

        R_cov = np.array([[sigma_heading**2]])

        return self.update(z, h_x, H, R_cov, update_type=source, alpha=alpha, timestamp=timestamp)

    def update_ai_forward_speed(
        self,
        speed_fwd: float,
        sigma_speed: float = 1.0,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """
        Update with AI estimated forward speed along Vehicle Y axis.
        v_veh = R_nav_to_veh * v_nav = R^T * v_nav.
        Forward speed is the 2nd component (index 1) of v_veh: y_axis_nav . v_nav.
        """
        R = self.quat_to_rot(self.q)
        # Vehicle Y axis in nav frame is the 2nd column of R
        y_axis_nav = R[:, 1]
        v_fwd_est = np.dot(y_axis_nav, self.v)

        z = np.array([speed_fwd])
        h_x = np.array([v_fwd_est])

        H = np.zeros((1, 15))
        # Derivative w.r.t delta_v: y_axis_nav
        H[0, 3:6] = y_axis_nav
        # Derivative w.r.t delta_theta: y_axis_nav x v_nav (small, can include or omit)
        H[0, 6:9] = np.cross(y_axis_nav, self.v)

        R_cov = np.array([[sigma_speed**2]])

        return self.update(z, h_x, H, R_cov, update_type="AI_SPEED", alpha=alpha, timestamp=timestamp)

    def update_map_matching_position(
        self,
        p_mm_enu: np.ndarray,
        sigma_pos: float = 3.0,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """
        Update with map-matching Position measurement in ENU.
        Used as pseudo-measurement during GNSS outage.
        """
        z = p_mm_enu  # (3,)
        h_x = self.p    # (3,)
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        R_cov = np.eye(3) * (sigma_pos**2)

        return self.update(z, h_x, H, R_cov, update_type="MAP_POS", alpha=alpha, timestamp=timestamp)

    def update_map_matching_heading(
        self,
        heading_rad: float,
        sigma_heading: float = np.radians(5.0),
        alpha: float = 0.01,
        timestamp: float = 0.0,
        source: str = "MAP_HEADING"
    ) -> Tuple[bool, float, float]:
        """
        Update with absolute Heading measurement from map-matching (geographic heading: 0 = North, +pi/2 = East, in rad).
        This mimics the magnetometer heading update but is gated by map-matching confidence.
        """
        return self.update_heading(
            heading_rad=heading_rad,
            sigma_heading=sigma_heading,
            alpha=alpha,
            timestamp=timestamp,
            source=source
        )

    def update_zupt(
        self,
        sigma_zupt: float = 0.05,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """
        Zero Velocity Update (ZUPT) when vehicle is stationary.
        z = [0, 0, 0] (m/s) in Nav frame.
        h(x) = v_nav
        H = [0_{3x3}, I_{3x3}, 0_{3x3}, 0_{3x3}, 0_{3x3}]
        """
        z = np.zeros(3)
        h_x = self.v
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        R_cov = np.eye(3) * (sigma_zupt**2)

        return self.update(z, h_x, H, R_cov, update_type="ZUPT", alpha=alpha, timestamp=timestamp)

    def update_zero_angular_rate(
        self,
        gyro_veh: np.ndarray,
        sigma_gyro_bias: float = 0.005,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """
        Zero Angular Rate Update (ZARU) when vehicle is stationary.
        Enforces gyro bias estimation when true angular rate is zero.
        z = gyro_veh
        h(x) = b_g
        H = [0_{3x12}, I_{3x3}]
        """
        z = np.copy(gyro_veh)
        h_x = np.copy(self.b_g)
        H = np.zeros((3, 15))
        H[0:3, 12:15] = np.eye(3)
        R_cov = np.eye(3) * (sigma_gyro_bias**2)

        return self.update(z, h_x, H, R_cov, update_type="ZARU", alpha=alpha, timestamp=timestamp)

    def update_nhc(
        self,
        vehicle_type: str = "car",
        lean_angle_rad: float = 0.0,
        sigma_nhc_x: float = 0.2,
        sigma_nhc_z: float = 0.2,
        alpha: float = 0.01,
        timestamp: float = 0.0
    ) -> Tuple[bool, float, float]:
        """
        Non-Holonomic Constraints (NHC) measurement update.
        For Car: lateral (X) and vertical (Z) velocities in vehicle frame are 0.
        For Two-Wheeler: lateral (X) and vertical (Z) velocities in lean-compensated road frame are 0 (N1).
        """
        R = self.quat_to_rot(self.q)
        v_veh = R.T @ self.v

        if vehicle_type == "two_wheeler" and abs(lean_angle_rad) > 1e-4:
            phi = lean_angle_rad
            c, s = np.cos(phi), np.sin(phi)
            # R_y: rotation around vehicle Y-axis (forward) by -phi
            R_y = np.array([
                [c, 0.0, s],
                [0.0, 1.0, 0.0],
                [-s, 0.0, c]
            ])
            M = R_y @ R.T  # (3, 3)
            N = R_y @ self.skew_symmetric(v_veh)  # (3, 3)
            v_meas = R_y @ v_veh
        else:
            M = R.T  # (3, 3)
            N = self.skew_symmetric(v_veh)  # (3, 3)
            v_meas = v_veh

        # Measurement: z = [0, 0] (lateral X, vertical Z)
        z = np.zeros(2)
        h_x = np.array([v_meas[0], v_meas[2]])

        H = np.zeros((2, 15))
        # Derivative w.r.t delta_v
        H[0, 3:6] = M[0, :]
        H[1, 3:6] = M[2, :]
        # Derivative w.r.t delta_theta
        H[0, 6:9] = N[0, :]
        H[1, 6:9] = N[2, :]

        R_cov = np.diag([sigma_nhc_x**2, sigma_nhc_z**2])

        return self.update(z, h_x, H, R_cov, update_type="NHC", alpha=alpha, timestamp=timestamp)

    def get_position_covariance_2d(self) -> np.ndarray:
        """Return 2x2 horizontal position covariance matrix (East, North)."""
        return self.P[0:2, 0:2]

    def get_euler_angles_deg(self) -> Tuple[float, float, float]:
        """Return Roll, Pitch, Yaw in degrees (Tait-Bryan ZYX convention)."""
        R = self.quat_to_rot(self.q)
        pitch = np.arcsin(np.clip(-R[2, 1], -1.0, 1.0))
        roll = np.arctan2(R[2, 0], R[2, 2])
        yaw = np.arctan2(R[0, 1], R[1, 1])  # Geographic yaw (0 = North, 90 = East)
        return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))
