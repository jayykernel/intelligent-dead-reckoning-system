"""
Lean-Angle EKF Estimator (N1)

Estimates the roll/lean angle of a two-wheeler in real-time
using gyro roll rates and specific force / kinematic-based lean inference.
"""

import numpy as np
from scipy import signal


class LeanAngleEKF:
    def __init__(self, dt: float = 0.1, q_phi: float = 1e-3, q_bias: float = 1e-5, r_meas: float = 0.1):
        self.dt = dt
        # State: [lean_angle (rad), roll_gyro_bias (rad/s)]
        self.x = np.zeros(2)
        # Covariance
        self.P = np.eye(2) * 0.01

        # Process noise covariance
        self.Q = np.diag([q_phi, q_bias])
        # Measurement noise covariance
        self.R = r_meas

        # Vibration notch filter for two-wheeler engine harmonics (10-20 Hz)
        # Design a second-order notch filter at 15 Hz with Q-factor of 5
        self.notch_freq = 15.0  # Hz
        self.notch_q = 5.0      # Quality factor
        fs = 1.0 / dt
        if fs > 2.0 * self.notch_freq:
            self.notch_b, self.notch_a = signal.iirnotch(self.notch_freq, self.notch_q, fs)
            self.notch_state = signal.lfilter_zi(self.notch_b, self.notch_a) * 0.0
            self._use_notch = True
        else:
            self._use_notch = False
        self.last_filtered_acc_x = 0.0

    def _validate_state(self):
        """Ensure state and covariance are finite."""
        if not np.all(np.isfinite(self.x)):
            raise ValueError("NaN/Inf in LeanAngleEKF state")
        if not np.all(np.isfinite(self.P)):
            raise ValueError("NaN/Inf in LeanAngleEKF covariance")

    def _ensure_positive_definite(self, epsilon: float = 1e-9):
        """Enforce symmetry and apply eigenvalue flooring to 2x2 covariance matrix."""
        if not np.all(np.isfinite(self.P)):
            raise ValueError("NaN/Inf in LeanAngleEKF covariance before positive-definite check")

        self.P = 0.5 * (self.P + self.P.T)
        try:
            w, v = np.linalg.eigh(self.P)
        except np.linalg.LinAlgError:
            self.P = np.eye(2) * epsilon
            return
        w_clipped = np.maximum(w, epsilon)
        self.P = v @ np.diag(w_clipped) @ v.T
        self.P = 0.5 * (self.P + self.P.T)

    def _apply_notch_filter(self, acc_x: float) -> float:
        """Apply notch filter to remove engine vibration from lateral acceleration."""
        if not getattr(self, '_use_notch', False):
            self.last_filtered_acc_x = acc_x
            return acc_x
        y, self.notch_state = signal.lfilter(self.notch_b, self.notch_a, [acc_x], zi=self.notch_state)
        self.last_filtered_acc_x = float(y[0])
        return float(y[0])

    def predict(self, gyro_y: float):
        """
        Propagate lean angle using forward-axis roll gyro.
        """
        if not np.isfinite(gyro_y):
            raise ValueError("NaN/Inf gyro_y input to LeanAngleEKF.predict")

        phi, b = self.x
        omega = gyro_y - b

        # State transition: phi_k+1 = phi_k + omega * dt
        self.x[0] = phi + omega * self.dt

        # Jacobian F = [[1, -dt], [0, 1]]
        F = np.array([[1.0, -self.dt], [0.0, 1.0]])
        self.P = F @ self.P @ F.T + self.Q
        self._ensure_positive_definite()
        self._validate_state()

        return self.x[0]

    def update(self, acc_x: float, acc_z: float, speed: float = 0.0, gyro_z: float = 0.0, g: float = 9.81):
        """
        Measurement update using apparent gravity vector and/or kinematic lean angle.
        Applies vibration notch filter to lateral acceleration for two-wheelers.
        """
        if not np.all(np.isfinite([acc_x, acc_z, speed, gyro_z, g])):
            raise ValueError("NaN/Inf input to LeanAngleEKF.update")

        # Apply vibration notch filter to lateral acceleration
        filtered_acc_x = self._apply_notch_filter(acc_x)

        # When moving, calculate kinematic expected roll vs static accel roll
        if speed > 1.0 and abs(gyro_z) > 0.05:
            # Centripetal acceleration balance: tan(phi) = v * omega_z / g
            phi_meas = np.arctan((speed * gyro_z) / g)
        else:
            # Low speed / stationary: use lateral vs vertical specific force
            phi_meas = np.arctan2(filtered_acc_x, acc_z)

        # Measurement Jacobian H = [1, 0]
        H = np.array([[1.0, 0.0]])
        z = phi_meas
        y = z - self.x[0] # Innovation

        # S = H @ P @ H^T + R
        S = float((H @ self.P @ H.T)[0, 0] + self.R)
        K = (self.P @ H.T) / S  # (2, 1)

        self.x = self.x + K.flatten() * y

        # Joseph form covariance update: P = (I - K H) P (I - K H)^T + K R K^T
        I_KH = np.eye(2) - K @ H
        R_cov = np.array([[self.R]])
        self.P = I_KH @ self.P @ I_KH.T + K @ R_cov @ K.T
        self._ensure_positive_definite()
        self._validate_state()

        return self.x[0]

    def get_lean_angle_deg(self) -> float:
        return np.degrees(self.x[0])
