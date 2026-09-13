"""
Error-State Kalman Filter (ESKF) for correcting INS drift using observations.

References:
    - Sola, "Quaternion kinematics for the error-state Kalman filter"
"""
from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np
from dataclasses import dataclass

from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState
from core.alignment.quaternion_utils import quat_from_axis_angle, quat_multiply, quat_normalize

@dataclass
class UpdateResult:
    """Result of an ESKF measurement update."""
    accepted: bool
    innovation: np.ndarray
    innovation_cov: np.ndarray
    mahalanobis_dist: float

class ErrorStateKalmanFilter:
    """
    ESKF wrap around the Strapdown INS.
    
    Responsible for:
    - Applying Kalman equations to measurement residuals.
    - Injecting error-state estimations into the nominal INS state.
    - Resetting the error state securely.
    """
    def __init__(self, ins: StrapdownINS):
        self.ins = ins

    def _apply_measurement(
        self,
        z: np.ndarray,
        H: np.ndarray,
        R: np.ndarray,
        mahalanobis_gate: float = 3.0
    ) -> UpdateResult:
        """
        Core Kalman update logic.
        """
        P = self.ins.covariance

        # Innovation covariance S = H*P*H^T + R
        S = H @ P @ H.T + R
        
        # Invert S safely (S should be positive definite)
        # Using pinv or solve for stability. For small dimensions, solve is fine.
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Fallback for numerical stability
            S_inv = np.linalg.pinv(S)

        # Mahalanobis distance check for outlier rejection
        mahalanobis = float(np.sqrt(np.clip(z.T @ S_inv @ z, 0.0, None)))
        
        if mahalanobis > mahalanobis_gate:
            return UpdateResult(accepted=False, innovation=z, innovation_cov=S, mahalanobis_dist=mahalanobis)

        # Kalman Gain K = P * H^T * S^-1
        K = P @ H.T @ S_inv

        # Error state update
        delta_x = K @ z

        # Covariance update (Joseph form for numeric stability and guaranteed PSD)
        I_KH = np.eye(15) - K @ H
        P_new = I_KH @ P @ I_KH.T + K @ R @ K.T
        P_new = 0.5 * (P_new + P_new.T) # force symmetry

        # Apply state injection
        self._inject_error_state(delta_x)

        # Update INS covariance
        self.ins.covariance = P_new

        return UpdateResult(accepted=True, innovation=z, innovation_cov=S, mahalanobis_dist=mahalanobis)

    def _inject_error_state(self, delta_x: np.ndarray) -> None:
        """
        Inject the 15-state error vector into the nominal non-linear INS state.
        State order: [p(3), v(3), theta(3), ba(3), bg(3)]
        """
        state = self.ins.state

        delta_p = delta_x[0:3]
        delta_v = delta_x[3:6]
        delta_t = delta_x[6:9]
        delta_ba = delta_x[9:12]
        delta_bg = delta_x[12:15]

        # 1. Position and Velocity (Additive)
        p_new = np.array(state.position_m) + delta_p
        v_new = np.array(state.velocity_mps) + delta_v

        # 2. Biases (Additive in sensor frame)
        ba_new = np.array(state.accel_bias_mps2) + delta_ba
        bg_new = np.array(state.gyro_bias_radps) + delta_bg

        # 3. Attitude (Multiplicative)
        angle = float(np.linalg.norm(delta_t))
        if angle > 1e-12:
            axis = delta_t / angle
            q_err = quat_from_axis_angle(tuple(axis), angle)
        else:
            q_err = (1.0, 0.0, 0.0, 0.0)

        # q_true = q_nom * q_err (because delta_theta was defined in local vehicle frame)
        q_new = quat_multiply(state.attitude_q_v2n, q_err)
        q_new = quat_normalize(q_new)

        # 4. Construct new state
        new_state = NavState(
            timestamp_ns=state.timestamp_ns,
            position_m=tuple(p_new),
            velocity_mps=tuple(v_new),
            attitude_q_v2n=q_new,
            accel_bias_mps2=tuple(ba_new),
            gyro_bias_radps=tuple(bg_new)
        )

        self.ins.state = new_state

    def update_position(
        self,
        pos_ned: Tuple[float, float, float],
        pos_cov: np.ndarray,
        gate: float = 5.0
    ) -> UpdateResult:
        """
        Update using position measurement in NED frame.
        """
        z = np.array(pos_ned) - np.array(self.ins.state.position_m)
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3) # dz / dp = I

        return self._apply_measurement(z, H, pos_cov, mahalanobis_gate=gate)

    def update_velocity(
        self,
        vel_ned: Tuple[float, float, float],
        vel_cov: np.ndarray,
        gate: float = 4.0
    ) -> UpdateResult:
        """
        Update using velocity measurement in NED frame.
        """
        z = np.array(vel_ned) - np.array(self.ins.state.velocity_mps)
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3) # dz / dv = I

        return self._apply_measurement(z, H, vel_cov, mahalanobis_gate=gate)

    def update_zero_velocity(self, vel_cov: np.ndarray = np.eye(3) * 0.01) -> UpdateResult:
        """
        Zero-Velocity Update (ZUPT).
        """
        # A specific case of velocity update where v_gnss = [0, 0, 0]
        return self.update_velocity((0.0, 0.0, 0.0), vel_cov, gate=float('inf'))

    def update_forward_velocity(
        self,
        forward_speed_mps: float,
        variance: float,
        gate: float = 3.0,
        couple_attitude: bool = False
    ) -> UpdateResult:
        """
        Update using a forward velocity measurement in the Vehicle frame (e.g. from ML model).

        Args:
            forward_speed_mps: Estimated speed along Vehicle X axis.
            variance: Measurement uncertainty variance.
            gate: Mahalanobis distance gate.
            couple_attitude: If True, include attitude cross-coupling in the Jacobian.
                             If False (default), set attitude Jacobian to zero for stability
                             during GNSS outages when attitude is not independently constrained.
        """
        from core.alignment.quaternion_utils import quat_to_rotation_matrix, quat_rotate_vector

        # Current nominal velocity in Navigation frame
        v_n = np.array(self.ins.state.velocity_mps)

        # Nominal mapping from Navigation to Vehicle frame: R_n2v = (R_v2n)^T
        q_v2n = self.ins.state.attitude_q_v2n
        R_v2n = quat_to_rotation_matrix(q_v2n)
        R_n2v = R_v2n.T

        # Expected velocity in Vehicle frame
        v_v = R_n2v @ v_n

        # Measurement is along Vehicle X axis
        expected_v_x = v_v[0]
        z = np.array([forward_speed_mps - expected_v_x])

        # Jacobian H (1 x 15)
        # delta_v_v = R_n2v @ delta_v_n + [v_v x] @ delta_theta
        H = np.zeros((1, 15))

        # d(v_x) / d(v_n) = Row 0 of R_n2v
        H[0, 3:6] = R_n2v[0, :]

        # d(v_x) / d(theta) = Row 0 of [v_v x]
        # [v_v x] = [[0, -vz, vy], [vz, 0, -vx], [-vy, vx, 0]]
        # Row 0 is [0, -v_v[2], v_v[1]]
        if couple_attitude:
            # Include attitude cross-coupling term
            H[0, 6:9] = np.array([0.0, -v_v[2], v_v[1]])
        else:
            # Set to zero to prevent instability and attitude divergence
            H[0, 6:9] = np.array([0.0, 0.0, 0.0])

        R_mat = np.array([[variance]])

        return self._apply_measurement(z, H, R_mat, mahalanobis_gate=gate)
