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
        # Rotate magnetic field vector into navigation frame: B_nav = R_veh_to_nav @ mag_veh
        B_nav = R_veh_to_nav @ mag_veh
        # B_nav = [B_East, B_North, B_Up]
        # In ENU: Magnetic North is in the horizontal plane (East, North).
        # Yaw angle psi (0 = East, pi/2 = North, or geographical heading):
        # In standard ENU navigation frame:
        # Geographic yaw psi = atan2(B_North, B_East) or geographical heading = atan2(B_East, B_North).
        # In EKF yaw convention: psi = atan2(R[1,0], R[0,0]) where vehicle forward Y is aligned with Nav North/East.
        # When vehicle Y points North (0 deg heading), B_veh points along Y.
        # B_nav horizontal angle:
        yaw_mag_rad = float(np.arctan2(B_nav[1], B_nav[0]))

        return True, yaw_mag_rad, debug
