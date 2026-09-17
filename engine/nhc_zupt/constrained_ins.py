"""
Constrained INS with NHC/ZUPT and Lean-Compensated NHC (N1)

Implements:
- Standard NHC (No Lateral Slide, No Vertical Velocity) for car/truck
- Lean-compensated NHC for two-wheeler (N1)
- ZUPT (Zero Velocity Update) for stopped vehicles (car/truck and two-wheeler)

Input: vehicle-frame specific force (acc_veh) and angular rate (gyro_veh) after calibration and alignment
Input: vehicle-type label from classifier ('car' or 'two_wheeler')
Input: lean angle estimate (in radians) from LeanAngleEKF (only used for two-wheeler)
Output: constrained velocity in vehicle frame (v_constrained)
"""

import numpy as np

class ConstrainedINS:
    def __init__(self, dt: float = 0.1, zupt_speed_threshold: float = 0.5, zupt_acc_threshold: float = 0.5, zupt_gyro_threshold: float = 0.1):
        self.dt = dt
        self.zupt_speed_threshold = zupt_speed_threshold
        self.zupt_acc_threshold = zupt_acc_threshold
        self.zupt_gyro_threshold = zupt_gyro_threshold
        self.g = 9.80665

    def _is_stopped(self, acc_veh: np.ndarray, gyro_veh: np.ndarray) -> bool:
        """
        Detect if the vehicle is stationary based on specific force and gyro.
        """
        acc_mag = np.linalg.norm(acc_veh)
        gyro_mag = np.linalg.norm(gyro_veh)
        return (abs(acc_mag - self.g) < self.zupt_acc_threshold) and (gyro_mag < self.zupt_gyro_threshold)

    def constrain(self, acc_veh: np.ndarray, gyro_veh: np.ndarray, v_veh: np.ndarray, vehicle_type: str, lean_angle_rad: float = 0.0) -> np.ndarray:
        """
        Apply NHC/ZUPT or lean-compensated NHC to the vehicle-frame velocity.

        Parameters:
            acc_veh: (3,) specific force in vehicle frame (m/s^2)
            gyro_veh: (3,) angular rate in vehicle frame (rad/s)
            v_veh: (3,) velocity in vehicle frame (m/s) [v_x, v_y, v_z]
            vehicle_type: 'car' or 'two_wheeler'
            lean_angle_rad: lean angle in radians (positive for right lean) - only used if vehicle_type == 'two_wheeler'

        Returns:
            v_constrained: (3,) constrained velocity in vehicle frame
        """
        # ZUPT: if stopped, set velocity to zero
        if self._is_stopped(acc_veh, gyro_veh):
            return np.zeros(3)

        if vehicle_type == 'car':
            # Standard NHC: no lateral slide, no vertical velocity
            v_constrained = v_veh.copy()
            v_constrained[1] = 0.0  # lateral (y)
            v_constrained[2] = 0.0  # vertical (z)
            return v_constrained

        elif vehicle_type == 'two_wheeler':
            # Lean-compensated NHC (N1)
            # Transform velocity to road frame by rotating around x-axis (forward) by -lean_angle
            # Road frame: x forward, y left, z up (when upright)
            # When leaned by phi (positive = right lean), the road frame is rotated by -phi around x from vehicle frame.
            phi = lean_angle_rad
            # Rotation matrix around x-axis by -phi
            R_x = np.array([
                [1, 0, 0],
                [0, np.cos(phi), np.sin(phi)],
                [0, -np.sin(phi), np.cos(phi)]
            ])
            # Transform to road frame
            v_road = R_x @ v_veh
            # Apply NHC in road frame: no lateral slide (y_road) and no vertical velocity (z_road)
            v_road_constrained = np.array([v_road[0], 0.0, 0.0])
            # Transform back to vehicle frame
            v_constrained = R_x.T @ v_road_constrained  # Note: R_x.T is rotation by +phi
            return v_constrained

        else:
            raise ValueError(f"Unknown vehicle type: {vehicle_type}")