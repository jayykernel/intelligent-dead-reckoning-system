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
        self.notch_b, self.notch_a = self._design_notch_filter(dt)
        self.notch_state = np.zeros(2)  # For direct form II implementation

    def _design_notch_filter(self, dt: float):
        """Design a second-order notch filter for removing engine vibration."""
        fs = 1.0 / dt  # Sampling frequency
        f0 = self.notch_freq / (fs / 2.0)  # Normalized frequency
        bw = f0 / self.notch_q  # Bandwidth

        # Second-order notch filter coefficients
        b0 = 1.0
        b1 = -2.0 * np.cos(2.0 * np.pi * f0)
        b2 = 1.0
        a0 = 1.0 + 2.0 * np.cos(2.0 * np.pi * f0) / (2.0 * self.notch_q)
        a1 = -2.0 * np.cos(2.0 * np.pi * f0)
        a2 = 1.0 - 2.0 * np.cos(2.0 * np.pi * f0) / (2.0 * self.notch_q)

        # Normalize
        b = np.array([b0, b1, b2]) / a0
        a = np.array([1.0, a1/a0, a2/a0])
        return b, a

    def _apply_notch_filter(self, acc_x: float) -> float:
        """Apply notch filter to remove engine vibration from lateral acceleration."""
        # Direct Form II implementation
        x = np.array([acc_x, 0.0, 0.0])
        y = np.dot(self.notch_b, x) - np.dot(self.notch_a[1:], self.notch_state)

        # Update state
        self.notch_state[1] = self.notch_state[0]
        self.notch_state[0] = y

        return y

    def predict(self, gyro_y: float):
        """
        Propagate lean angle using forward-axis roll gyro.
        """
        phi, b = self.x
        omega = gyro_y - b

        # State transition: phi_k+1 = phi_k + omega * dt
        self.x[0] = phi + omega * self.dt

        # Jacobian F = [[1, -dt], [0, 1]]
        F = np.array([[1.0, -self.dt], [0.0, 1.0]])
        self.P = F @ self.P @ F.T + self.Q

        return self.x[0]

    def update(self, acc_x: float, acc_z: float, speed: float = 0.0, gyro_z: float = 0.0, g: float = 9.81):
        """
        Measurement update using apparent gravity vector and/or kinematic lean angle.
        Applies vibration notch filter to lateral acceleration for two-wheelers.
        """
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

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T / S

        self.x = self.x + K.flatten() * y
        self.P = (np.eye(2) - K @ H) @ self.P

        return self.x[0]

    def get_lean_angle_deg(self) -> float:
        return np.degrees(self.x[0])
