"""
engine/fusion/mag_gate.py

Magnetometer Disturbance Gating (N5).

Monitors raw 3D magnetic field measurements for ferromagnetic anomalies,
cabin electronics interference, or abrupt step disturbances.
Gates off the magnetometer entirely when disturbed, falling back to pure
gyro integration and GNSS course heading.
"""

import numpy as np
from typing import Dict, Tuple, Optional, List


class MagnetometerGate:
    def __init__(
        self,
        ref_field_strength: float = 48.0,  # Expected Earth geomagnetic field magnitude (uT)
        norm_tolerance: float = 15.0,       # Max allowed deviation from ref_strength (uT)
        gradient_threshold: float = 8.0,    # Max allowed step change between consecutive samples (uT)
        window_size: int = 5,               # Window for rolling variance / disturbance detection
        variance_threshold: float = 25.0    # Max allowed variance over window
    ):
        self.ref_field_strength = ref_field_strength
        self.norm_tolerance = norm_tolerance
        self.gradient_threshold = gradient_threshold
        self.window_size = window_size
        self.variance_threshold = variance_threshold

        self.mag_history: List[np.ndarray] = []
        self.last_norm: Optional[float] = None
        self.is_disturbed: bool = False
        self.disturbance_reason: str = "INITIALIZING"

    def process_measurement(
        self,
        mag_veh: np.ndarray,
        R_veh_to_nav: np.ndarray
    ) -> Tuple[bool, Optional[float], Dict]:
        """
        Process incoming magnetic field measurement in vehicle frame (uT).

        mag_veh: (3,) Magnetic field vector in vehicle frame [Bx, By, Bz]
        R_veh_to_nav: (3, 3) Current estimated rotation matrix

        Returns:
        (is_valid_and_usable, tilt_compensated_yaw_rad, debug_info)
        """
        mag_norm = float(np.linalg.norm(mag_veh))
        self.mag_history.append(mag_veh)
        if len(self.mag_history) > self.window_size:
            self.mag_history.pop(0)

        debug = {
            "mag_norm": mag_norm,
            "ref_strength": self.ref_field_strength,
            "is_disturbed": False,
            "reason": "OK"
        }

        # 1. Check absolute field magnitude deviation
        norm_dev = abs(mag_norm - self.ref_field_strength)
        if norm_dev > self.norm_tolerance:
            self.is_disturbed = True
            self.disturbance_reason = f"Norm anomaly ({mag_norm:.1f} uT vs ref {self.ref_field_strength:.1f} uT, dev={norm_dev:.1f})"
            debug["is_disturbed"] = True
            debug["reason"] = self.disturbance_reason
            self.last_norm = mag_norm
            return False, None, debug

        # 2. Check gradient / step change
        if self.last_norm is not None:
            gradient = abs(mag_norm - self.last_norm)
            if gradient > self.gradient_threshold:
                self.is_disturbed = True
                self.disturbance_reason = f"Step disturbance detected ({gradient:.1f} uT/sample)"
                debug["is_disturbed"] = True
                debug["reason"] = self.disturbance_reason
                self.last_norm = mag_norm
                return False, None, debug

        self.last_norm = mag_norm

        # 3. Check short-term vector stability / variance
        if len(self.mag_history) >= self.window_size:
            mats = np.array(self.mag_history)
            variance = np.sum(np.var(mats, axis=0))
            if variance > self.variance_threshold:  # High local magnetic noise
                self.is_disturbed = True
                self.disturbance_reason = f"High local magnetic variance ({variance:.1f})"
                debug["is_disturbed"] = True
                debug["reason"] = self.disturbance_reason
                return False, None, debug

        # If all checks pass, magnetometer is clean
        self.is_disturbed = False
        self.disturbance_reason = "CLEAN"

        # 4. Compute tilt-compensated magnetic heading in Nav frame (ENU)
        # Key: R_veh_to_nav contains yaw drift. We must level mag_veh using ONLY
        # the gravity direction from the attitude estimate, completely independent of yaw.
        #
        # In vehicle frame, the unit Up vector (opposite to gravity) is Row 2 of R_veh_to_nav:
        # u_veh = R_veh_to_nav^T @ [0, 0, 1]^T = [R[2,0], R[2,1], R[2,2]]^T
        # Row 2 of R_veh_to_nav = R_z(yaw) @ R_tilt has ZERO dependence on yaw.
        u_veh = R_veh_to_nav[2, :].copy()
        u_norm = np.linalg.norm(u_veh)
        if u_norm < 1e-6:
            u_veh = np.array([0.0, 0.0, 1.0])
        else:
            u_veh = u_veh / u_norm

        # Project vehicle forward axis Y_veh = [0, 1, 0] onto horizontal plane (orthogonal to u_veh)
        u_y = u_veh[1]
        y_proj = np.array([-u_y * u_veh[0], 1.0 - u_y**2, -u_y * u_veh[2]])
        y_norm = np.linalg.norm(y_proj)
        if y_norm < 1e-6:
            # Vehicle pointing straight up/down (gimbal lock fallback)
            y_level = np.array([0.0, 1.0, 0.0])
        else:
            y_level = y_proj / y_norm

        # Leveled X axis (Right) = Y_level x Z_level (u_veh)
        x_level = np.cross(y_level, u_veh)
        x_norm = np.linalg.norm(x_level)
        if x_norm > 1e-6:
            x_level = x_level / x_norm

        # Rotation matrix from vehicle frame to leveled horizontal frame:
        # Rows are the leveled basis vectors in vehicle coordinates
        R_veh_to_level = np.vstack([x_level, y_level, u_veh])

        # Level the magnetometer vector to horizontal plane
        B_level = R_veh_to_level @ mag_veh

        # Geographic heading psi (0 = North, +pi/2 = East) from horizontal components:
        # In ENU, Earth B points North (B_E = 0, B_N > 0).
        # When vehicle faces heading psi:
        # B_level,y (forward) = B_N cos(psi)
        # B_level,x (right)   = -B_N sin(psi)
        # Therefore: psi = atan2(-B_level,x, B_level,y)
        yaw_mag_rad = float(np.arctan2(-B_level[0], B_level[1]))

        return True, yaw_mag_rad, debug
