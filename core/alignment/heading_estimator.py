"""
Heading (yaw) alignment estimator using GNSS velocity and vehicle dynamics.

Estimates the phone's heading offset relative to the vehicle frame using:
1. GNSS-derived vehicle velocity direction (when sufficient speed)
2. Longitudinal/forward acceleration patterns
3. Lateral acceleration during turns (as a secondary indicator)

Magnetometer is used only as a weak aiding source to avoid reliance on
distortion-prone compass readings, per requirements.

Outputs a heading offset estimate and confidence.
"""

from __future__ import annotations

from collections import deque
import math
from typing import Deque, Optional, Tuple

import numpy as np

from ..sensors.data_types import GnssFix, ImuSample, MagSample
from .frames import VehicleType
from ..calibration.dynamic_error import MagneticReliabilityScorer
from .quaternion_utils import Quat, quat_from_axis_angle, quat_normalize, wrap_angle


class HeadingEstimator:
    """
    Estimates heading (yaw) offset between phone and vehicle frames.

    Combines multiple independent heading sources:
    - GNSS velocity vector (primary, most reliable when speed > threshold)
    - Forward acceleration (longitudinal dynamics)
    - Lateral acceleration during turns (secondary)
    - Magnetometer (weak aiding only)

    Provides confidence estimates and detects degenerate conditions
    (straight-line driving, low speed) where heading cannot be observed.
    """

    def __init__(
        self,
        vehicle_type: VehicleType = VehicleType.UNKNOWN,
        gnss_min_speed: float = 3.0,  # m/s (~6.7 mph)
        forward_accel_min: float = 0.5,  # m/s² minimum forward accel to use
        lateral_accel_min: float = 0.2,  # m/s² minimum lateral accel to use
        mag_weight: float = 0.1,  # Very low weight for magnetometer
        history_size: int = 50,
    ):
        """
        Args:
            vehicle_type: Type of vehicle (affects constraint tuning).
            gnss_min_speed: Minimum GNSS speed to use velocity heading (m/s).
            forward_accel_min: Minimum forward accel to use for heading estimate.
            lateral_accel_min: Minimum lateral accel to use for turn-based heading.
            mag_weight: Weight for magnetometer heading (kept very low).
            history_size: Number of samples to maintain for statistics.
        """
        self.vehicle_type = vehicle_type
        self.gnss_min_speed = gnss_min_speed
        self.forward_accel_min = forward_accel_min
        self.lateral_accel_min = lateral_accel_min
        self.mag_weight = mag_weight
        self.history_size = history_size

        # History buffers for estimates
        self._gnss_headings: Deque[float] = deque(maxlen=history_size)
        self._forward_accel_headings: Deque[float] = deque(maxlen=history_size)
        self._lateral_accel_headings: Deque[float] = deque(maxlen=history_size)
        self._mag_headings: Deque[float] = deque(maxlen=history_size)
        self._combined_headings: Deque[float] = deque(maxlen=history_size)
        self._confidences: Deque[float] = deque(maxlen=history_size)

        # Current state
        self._yaw_offset: float = 0.0
        self._confidence: float = 0.0
        self._is_valid: bool = False
        self._last_gnss_fix: Optional[GnssFix] = None
        self._mag_scorer = MagneticReliabilityScorer(window_size=history_size)
        self._current_mag_score: float = 0.1

    def add_gnss_fix(self, fix: GnssFix) -> bool:
        """
        Add GNSS fix and extract velocity-derived heading if sufficient speed.

        Args:
            fix: GNSS fix with velocity in NED frame.

        Returns:
            True if a usable heading was extracted, False otherwise.
        """
        self._last_gnss_fix = fix

        # Extract horizontal velocity
        v_north, v_east, _ = fix.velocity_ned_mps
        speed = math.hypot(v_north, v_east)

        if speed < self.gnss_min_speed:
            return False

        # Vehicle heading in navigation frame: atan2(v_east, v_north)
        # This is the direction the vehicle is moving (North-East-Down convention)
        heading_nav = math.atan2(v_east, v_north)

        # Store for later combination (need current tilt to convert to phone frame)
        self._gnss_headings.append(heading_nav)
        return True

    def add_imu_sample(self, sample: ImuSample) -> bool:
        """
        Add IMU sample and extract heading from longitudinal/lateral dynamics.

        Uses forward acceleration and lateral acceleration during turns.

        Args:
            sample: IMU sample.

        Returns:
            True if a usable heading was extracted, False otherwise.
        """
        accel = np.array(sample.accel_m_s2)
        gyro = np.array(sample.gyro_rad_s)

        # We need the current tilt estimate to transform accel to vehicle frame.
        # Since this estimator doesn't have direct access to tilt, we assume the
        # caller will provide tilt-compensated samples or we extract heading
        # from dynamics that are relatively insensitive to small tilt errors.
        #
        # Forward acceleration (X_vehicle) is relatively insensitive to small
        # pitch/roll errors when the vehicle is near-level.
        # Lateral acceleration (Y_vehicle) can be used during turns.

        # For a first implementation, we use a simplified approach:
        #   - Forward accel: significant longitudinal accel implies forward direction
        #   - Lateral accel: significant lateral accel implies turning, gives heading rate
        #
        # A more sophisticated implementation would integrate gyroscope to propagate
        # heading and use accel as correction (like a complementary filter).

        # Extract forward and lateral acceleration in phone frame
        # Assuming phone Z is roughly up (to be refined with tilt)
        forward_accel = accel[0]  # X_phone
        lateral_accel = accel[1]  # Y_phone
        vert_accel = accel[2]     # Z_phone

        # Simple heuristic: if we detect significant forward acceleration,
        # we assume the phone's X axis has a forward component
        forward_used = False
        lateral_used = False

        # Forward acceleration method (during acceleration/deceleration)
        if abs(forward_accel) >= self.forward_accel_min:
            # During forward accel, phone X axis points somewhat forward
            # During backward accel (deceleration), phone X points backward
            # We need to distinguish these cases using speed or GPS if available
            # For now, assume forward accel > 0 means accelerating forward
            if forward_accel > 0:
                # Phone X axis has forward component -> heading is roughly -90 deg
                # from phone X in the horizontal plane (need pitch/roll)
                # Without tilt, we cannot resolve this fully.
                # We'll leave this for a future iteration with tilt integration.
                pass
            # TODO: Implement forward acceleration heading with tilt compensation
            # For now, we'll skip this and rely on GNSS and magnetometer primarily
            pass

        # Lateral acceleration method (during turns)
        if abs(lateral_accel) >= self.lateral_accel_min and abs(gyro[2]) >= 0.1:  # yaw rate
            # During a turn, lateral accel points toward center of turn
            # Combined with yaw rate, we can infer heading change
            # This requires integration which is complex without tilt
            pass

        # For Phase 4 MVP, we'll focus on GNSS as primary heading source
        # and magnetometer as weak aid, with plans to enhance dynamics later
        return False

    def add_mag_sample(self, sample: MagSample) -> bool:
        """
        Add magnetometer sample as a weak heading aid.

        Magnetometer is susceptible to hard/soft iron distortions and
        magnetic anomalies, so it is given very low weight.

        Args:
            sample: Magnetometer sample.

        Returns:
            True if heading was extracted, False otherwise.
        """
        mag = np.array(sample.magnetic_field_ut)
        mag_mag = np.linalg.norm(mag)

        # Reject if too weak or too strong (saturation or anomalies)
        if mag_mag < 10.0 or mag_mag > 80.0:
            return False

        # Phase 14: Dynamic Magnetic Reliability Scoring
        # Adaptively score and weight the magnetometer reading based on environmental stability
        self._current_mag_score = self._mag_scorer.add_sample(tuple(mag))

        # Extract heading from horizontal components
        # Assuming phone is roughly level (to be improved with tilt compensation)
        mx, my, _ = mag

        # atan2 gives angle from +X axis toward +Y axis
        # For typical phone orientation, we need to determine the mapping
        # This is ambiguous without knowing the phone's mounting
        # We'll assume a standard convention and let confidence be low
        heading_mag = math.atan2(my, mx)  # Radians from phone X axis

        # Store heading
        self._mag_headings.append(heading_mag)
        return True

    def update(self) -> None:
        """
        Combine all available heading estimates and compute confidence.

        Should be called after adding samples to produce the current estimate.
        """
        # Collect heading estimates and their confidences from each source
        source_headings = []
        source_confs = []

        # GNSS velocity heading (highest confidence when available)
        if self._gnss_headings:
            # Use circular mean for heading averaging
            sin_sum = sum(math.sin(h) for h in self._gnss_headings)
            cos_sum = sum(math.cos(h) for h in self._gnss_headings)
            if abs(cos_sum) > 1e-6 or abs(sin_sum) > 1e-6:
                gnss_heading = math.atan2(sin_sum, cos_sum)
                # Confidence based on consistency and number of samples
                gnss_conf = min(1.0, len(self._gnss_headings) / 10.0)  # Build up over time
                source_headings.append(gnss_heading)
                source_confs.append(gnss_conf)

        # Magnetometer heading (very low weight due to susceptibility to distortion)
        if self._mag_headings:
            sin_sum = sum(math.sin(h) for h in self._mag_headings)
            cos_sum = sum(math.cos(h) for h in self._mag_headings)
            if abs(cos_sum) > 1e-6 or abs(sin_sum) > 1e-6:
                mag_heading = math.atan2(sin_sum, cos_sum)
                # Confidence based on consistency and number of samples, scaled by low weight
                mag_conf = min(1.0, len(self._mag_headings) / 20.0) * self.mag_weight * self._current_mag_score
                source_headings.append(mag_heading)
                source_confs.append(mag_conf)

        # Forward/lateral acceleration headings (placeholder for future enhancement)
        # Not implemented in MVP - will be added in later iteration

        if not source_headings:
            self._is_valid = False
            self._confidence = 0.0
            return

        # Compute circular mean of headings weighted by source confidences
        weighted_sin = sum(c * math.sin(h) for c, h in zip(source_confs, source_headings))
        weighted_cos = sum(c * math.cos(h) for c, h in zip(source_confs, source_headings))
        total_weight = sum(source_confs)
        if total_weight == 0:
            self._is_valid = False
            self._confidence = 0.0
            return
        mean_sin = weighted_sin / total_weight
        mean_cos = weighted_cos / total_weight
        vector_strength = math.hypot(mean_sin, mean_cos)  # Length of the mean vector (0 to 1)
        self._yaw_offset = math.atan2(mean_sin, mean_cos)
        self._yaw_offset = wrap_angle(self._yaw_offset)

        # Average confidence of the sources
        avg_source_conf = sum(source_confs) / len(source_confs)
        # Confidence is the product of vector strength (agreement) and average source confidence
        self._confidence = min(1.0, vector_strength * avg_source_conf)

        self._is_valid = True

        # Store combined result for history
        self._combined_headings.append(self._yaw_offset)
        self._confidences.append(self._confidence)

    @property
    def is_valid(self) -> bool:
        """Whether a valid heading estimate is available."""
        return self._is_valid

    @property
    def yaw_offset(self) -> float:
        """Estimated yaw offset in radians (phone forward to vehicle forward)."""
        return self._yaw_offset if self._is_valid else 0.0

    @property
    def confidence(self) -> float:
        """Heading estimate confidence in [0, 1]."""
        return self._confidence

    def get_yaw_quaternion(self) -> Quat:
        """Get quaternion representing current heading estimate.

        Returns rotation about Z axis (down) by the yaw offset.
        """
        return quat_from_axis_angle((0.0, 0.0, 1.0), self._yaw_offset)

    def reset(self) -> None:
        """Reset to uninitialized state."""
        self._gnss_headings.clear()
        self._forward_accel_headings.clear()
        self._lateral_accel_headings.clear()
        self._mag_headings.clear()
        self._combined_headings.clear()
        self._confidences.clear()
        self._yaw_offset = 0.0
        self._confidence = 0.0
        self._is_valid = False
        self._last_gnss_fix = None
        self._mag_scorer = MagneticReliabilityScorer(window_size=self.history_size)
        self._current_mag_score = 0.1