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
from engine.config.constants import GRAVITY_MS2

class ConstrainedINS:
    def __init__(self, dt: float = 0.1, zupt_speed_threshold: float = 0.5, zupt_acc_threshold: float = 0.5, zupt_gyro_threshold: float = 0.1):
        self.dt = dt
        self.zupt_speed_threshold = zupt_speed_threshold
        self.zupt_acc_threshold = zupt_acc_threshold
        self.zupt_gyro_threshold = zupt_gyro_threshold
        self.g = GRAVITY_MS2

    def _is_stopped(self, acc_veh: np.ndarray, gyro_veh: np.ndarray, v_veh: np.ndarray = None, ai_speed: float = None, vehicle_type: str = 'car') -> bool:
        """
        Detect if the vehicle is stationary based on specific force, gyro, current velocity, and ai_speed.
        """
        if ai_speed is not None and ai_speed > 0.5 and vehicle_type != 'two_wheeler':
            return False

        acc_mag = np.linalg.norm(acc_veh)
        gyro_mag = np.linalg.norm(gyro_veh)

        # For two-wheelers, engine idling vibration causes large acc/gyro noise,
        # so we relax the threshold significantly to ensure it ZUPTs when stopped.
        if vehicle_type == 'two_wheeler':
            acc_thresh = 2.0  # 2.0 m/s^2 vibration is common for a stopped motorcycle
            gyro_thresh = 0.5 # 0.5 rad/s vibration is common
        else:
            acc_thresh = self.zupt_acc_threshold
            gyro_thresh = self.zupt_gyro_threshold

        imu_quiet = (abs(acc_mag - self.g) < acc_thresh) and (gyro_mag < gyro_thresh)

        if v_veh is not None:
            v_mag = np.linalg.norm(v_veh)
            # Relax the velocity check for two-wheeler as well
            v_thresh = self.zupt_speed_threshold if vehicle_type != 'two_wheeler' else 1.5
            return imu_quiet and (v_mag < v_thresh)

        return imu_quiet

    def constrain(self, acc_veh: np.ndarray, gyro_veh: np.ndarray, v_veh: np.ndarray, vehicle_type: str, lean_angle_rad: float = 0.0) -> np.ndarray:
        """
        Apply NHC/ZUPT or lean-compensated NHC to the vehicle-frame velocity.
        Includes lateral acceleration sanity gate for two-wheelers to prevent NHC during sliding/weaving.
        """
        # ZUPT: if stopped, set velocity to zero
        if self._is_stopped(acc_veh, gyro_veh, v_veh):
            return np.zeros(3)

        if vehicle_type == 'car':
            # Standard NHC (Vehicle Frame: X Right, Y Forward, Z Up)
            # No lateral slide (x = 0), no vertical velocity (z = 0)
            v_constrained = v_veh.copy()
            v_constrained[0] = 0.0  # lateral (x)
            v_constrained[2] = 0.0  # vertical (z)
            return v_constrained

        elif vehicle_type == 'two_wheeler':
            # Lateral acceleration sanity gate: reject NHC update if lateral accel exceeds threshold
            # (indicates sliding/weaving, not pure cornering)
            lateral_acc = abs(acc_veh[0])  # x is lateral in vehicle frame
            # Threshold: 2.0 m/s^2 (adjustable based on typical cornering lateral accel)
            if lateral_acc > 2.0:
                # Skip NHC constraints; return unconstrained velocity (let INS drift)
                return v_veh.copy()

            # Lean-compensated NHC (N1)
            # Forward is Y-axis. Roll/lean rotation is around Y-axis by -lean_angle.
            phi = lean_angle_rad
            # Rotation matrix around Y-axis by -phi
            c, s = np.cos(phi), np.sin(phi)
            R_y = np.array([
                [c, 0.0, s],
                [0.0, 1.0, 0.0],
                [-s, 0.0, c]
            ])
            # Transform to road frame
            v_road = R_y @ v_veh
            # Apply NHC in road frame: no lateral slide (x_road = 0) and no vertical velocity (z_road = 0)
            v_road_constrained = np.array([0.0, v_road[1], 0.0])
            # Transform back to vehicle frame
            v_constrained = R_y.T @ v_road_constrained
            return v_constrained

        else:
            raise ValueError(f"Unknown vehicle type: {vehicle_type}")