"""
Deterministic Strapdown Inertial Navigation System (INS) Mechanization Engine.

Consumes calibrated and aligned IMU samples to update the full navigation state:
Position, Velocity, Attitude (quaternion), and Bias states.
Optionally propagates the 15-state covariance matrix using continuous-discrete
mechanization kinematics.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np

from core.sensors.data_types import ImuSample
from core.alignment.quaternion_utils import (
    Quat,
    quat_multiply,
    quat_normalize,
    quat_rotate_vector,
    quat_to_rotation_matrix,
    quat_from_axis_angle,
)
from .state import NavState

GRAVITY_STANDARD_MPS2 = 9.80665

def skew_symmetric(v: np.ndarray | Tuple[float, float, float]) -> np.ndarray:
    """Return the 3x3 skew-symmetric matrix [v x]."""
    x, y, z = v
    return np.array([
        [ 0.0, -z,   y],
        [ z,   0.0, -x],
        [-y,   x,   0.0]
    ])

class StrapdownINS:
    """
    Deterministic strapdown inertial navigation propagation engine.
    """

    def __init__(
        self,
        initial_state: Optional[NavState] = None,
        initial_covariance: Optional[np.ndarray] = None,
        accel_noise_std: float = 0.05,
        gyro_noise_std: float = 0.005,
        accel_bias_walk_std: float = 0.0001,
        gyro_bias_walk_std: float = 1e-5,
        gravity_mps2: float = GRAVITY_STANDARD_MPS2,
    ):

        self._F = np.zeros((15, 15))
        self._F[0:3, 3:6] = np.eye(3)
        self._Phi = np.eye(15)
        self._I15 = np.eye(15)
        self._Q_d = np.zeros((15, 15))
        self.state = initial_state if initial_state is not None else NavState.create_initial(0)
        
        if initial_covariance is not None:
            self.covariance = initial_covariance.copy()
        else:
            self.covariance = np.diag([
                1.0, 1.0, 1.0,
                0.1, 0.1, 0.1,
                0.01, 0.01, 0.01,
                0.001, 0.001, 0.001,
                1e-5, 1e-5, 1e-5
            ])
            
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std
        self.accel_bias_walk_std = accel_bias_walk_std
        self.gyro_bias_walk_std = gyro_bias_walk_std
        self.gravity_mps2 = gravity_mps2


    def clone(self) -> 'StrapdownINS':
        import copy
        new_ins = StrapdownINS(
            initial_state=copy.copy(self.state),
            initial_covariance=self.covariance.copy(),
            accel_noise_std=self.accel_noise_std,
            gyro_noise_std=self.gyro_noise_std,
            accel_bias_walk_std=self.accel_bias_walk_std,
            gyro_bias_walk_std=self.gyro_bias_walk_std,
            gravity_mps2=self.gravity_mps2
        )
        return new_ins

    def propagate(
        self,
        imu_sample: ImuSample,
        q_sensor_to_vehicle: Quat = (1.0, 0.0, 0.0, 0.0),
        propagate_covariance: bool = True
    ) -> NavState:
        dt = (imu_sample.timestamp_ns - self.state.timestamp_ns) * 1e-9
        if dt <= 0.0:
            self.state = NavState(
                timestamp_ns=imu_sample.timestamp_ns,
                position_m=self.state.position_m,
                velocity_mps=self.state.velocity_mps,
                attitude_q_v2n=self.state.attitude_q_v2n,
                accel_bias_mps2=self.state.accel_bias_mps2,
                gyro_bias_radps=self.state.gyro_bias_radps
            )
            return self.state

        f_s = np.array(imu_sample.accel_m_s2) - np.array(self.state.accel_bias_mps2)
        omega_s = np.array(imu_sample.gyro_rad_s) - np.array(self.state.gyro_bias_radps)

        f_v = np.array(quat_rotate_vector(q_sensor_to_vehicle, tuple(f_s)))
        omega_v = np.array(quat_rotate_vector(q_sensor_to_vehicle, tuple(omega_s)))

        delta_theta = omega_v * dt
        angle = float(np.linalg.norm(delta_theta))
        if angle > 1e-12:
            axis = delta_theta / angle
            q_rot = quat_from_axis_angle(tuple(axis), angle)
        else:
            q_rot = (1.0, 0.0, 0.0, 0.0)

        q_v2n_new = quat_multiply(self.state.attitude_q_v2n, q_rot)
        q_v2n_new = quat_normalize(q_v2n_new)

        f_n = np.array(quat_rotate_vector(self.state.attitude_q_v2n, tuple(f_v)))
        g_ned = np.array([0.0, 0.0, self.gravity_mps2])
        a_n = f_n + g_ned

        v_n = np.array(self.state.velocity_mps)
        p_n = np.array(self.state.position_m)

        v_n_new = v_n + a_n * dt
        p_n_new = p_n + v_n * dt + 0.5 * a_n * (dt ** 2)

        if propagate_covariance:
            self._propagate_covariance(dt, f_v, omega_v, q_sensor_to_vehicle)

        self.state = NavState(
            timestamp_ns=imu_sample.timestamp_ns,
            position_m=tuple(p_n_new),
            velocity_mps=tuple(v_n_new),
            attitude_q_v2n=q_v2n_new,
            accel_bias_mps2=self.state.accel_bias_mps2,
            gyro_bias_radps=self.state.gyro_bias_radps,
        )

        return self.state

    def _propagate_covariance(
        self,
        dt: float,
        f_v: tuple,
        omega_v: tuple,
        q_s2v: Quat
    ) -> None:
        R_v2n = quat_to_rotation_matrix(self.state.attitude_q_v2n)
        R_s2v = quat_to_rotation_matrix(q_s2v)
        R_s2n = R_v2n @ R_s2v

        # Update F safely retaining preallocated zero regions
        self._F[3:6, 6:9] = -R_v2n @ skew_symmetric(f_v)
        self._F[3:6, 9:12] = -R_s2n
        self._F[6:9, 6:9] = -skew_symmetric(omega_v)
        self._F[6:9, 12:15] = -R_s2v

        # Phi = I + F * dt
        np.copyto(self._Phi, self._I15)
        self._Phi += self._F * dt
        
        # Efficient Q_d direct assignment
        np.fill_diagonal(self._Q_d, 0.0) # flush
        dt_a = (self.accel_noise_std ** 2) * dt
        dt_g = (self.gyro_noise_std ** 2) * dt
        dt_ba = (self.accel_bias_walk_std ** 2) * dt
        dt_bg = (self.gyro_bias_walk_std ** 2) * dt
        
        for i in range(3, 6): self._Q_d[i, i] = dt_a
        for i in range(6, 9): self._Q_d[i, i] = dt_g
        for i in range(9, 12): self._Q_d[i, i] = dt_ba
        for i in range(12, 15): self._Q_d[i, i] = dt_bg

        self.covariance = self._Phi @ self.covariance @ self._Phi.T + self._Q_d
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def set_biases(
        self,
        accel_bias_mps2: Tuple[float, float, float],
        gyro_bias_radps: Tuple[float, float, float]
    ) -> None:
        self.state = NavState(
            timestamp_ns=self.state.timestamp_ns,
            position_m=self.state.position_m,
            velocity_mps=self.state.velocity_mps,
            attitude_q_v2n=self.state.attitude_q_v2n,
            accel_bias_mps2=accel_bias_mps2,
            gyro_bias_radps=gyro_bias_radps,
        )

    def reset(self, new_state: Optional[NavState] = None) -> None:
        self.state = new_state if new_state is not None else NavState.create_initial(0)
