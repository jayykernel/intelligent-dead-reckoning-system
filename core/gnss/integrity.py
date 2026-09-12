"""
GNSS Integrity and Quality Assessment Engine.

Provides continuous GNSS quality scoring, classification (TRUSTED, DEGRADED, REJECTED),
adaptive measurement covariance scaling, and innovation-based gating.

Uses only physically available fields from GnssFix:
- horizontal_accuracy_m, vertical_accuracy_m, speed_accuracy_mps
- satellite_count
- velocity_ned_mps, latitude_deg, longitude_deg, altitude_m
- timestamp_ns
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Tuple
import numpy as np

from core.sensors.data_types import GnssFix


class GnssClassification(Enum):
    """Classification of GNSS measurement trustworthiness."""
    TRUSTED = auto()     # High quality, use standard covariance
    DEGRADED = auto()    # Moderate quality, inflate covariance
    REJECTED = auto()    # Unusable / Outlier, do not apply to filter


@dataclass(frozen=True)
class GnssIntegrityReport:
    """Detailed report of GNSS integrity assessment.

    Attributes:
        quality_score: Continuous quality metric in [0.0, 1.0].
        classification: Categorical trustworthiness (TRUSTED, DEGRADED, REJECTED).
        position_accuracy_score: Sub-score for position accuracy [0, 1].
        velocity_accuracy_score: Sub-score for velocity accuracy [0, 1].
        satellite_score: Sub-score for satellite count [0, 1].
        innovation_score: Sub-score for innovation consistency [0, 1].
        temporal_score: Sub-score for temporal consistency / jump detection [0, 1].
        scaled_pos_cov: 3x3 position covariance matrix scaled by quality.
        scaled_vel_cov: 3x3 velocity covariance matrix scaled by quality.
        rejection_reason: Human-readable reason if REJECTED, else empty string.
        position_jump_m: Detected position jump from previous fix (m), or None.
        time_delta_s: Time elapsed since previous fix (s), or None.
    """
    quality_score: float
    classification: GnssClassification
    position_accuracy_score: float
    velocity_accuracy_score: float
    satellite_score: float
    innovation_score: float
    temporal_score: float
    scaled_pos_cov: np.ndarray
    scaled_vel_cov: np.ndarray
    rejection_reason: str = ""
    position_jump_m: Optional[float] = None
    time_delta_s: Optional[float] = None


@dataclass
class GnssIntegrityConfig:
    """Configurable thresholds for GNSS integrity assessment.

    All thresholds are physically motivated and mathematically defensible.
    """
    # Quality score thresholds for classification
    trusted_threshold: float = 0.70      # Quality score >= this -> TRUSTED
    degraded_threshold: float = 0.35     # Quality score >= this -> DEGRADED, below -> REJECTED

    # Accuracy thresholds (1-sigma)
    max_acceptable_hacc_m: float = 25.0  # Beyond this, position accuracy score -> 0
    nominal_hacc_m: float = 3.0          # Below this, position accuracy score -> 1.0
    max_acceptable_vacc_m: float = 35.0  # Beyond this, vertical accuracy score -> 0
    nominal_vacc_m: float = 5.0          # Below this, vertical accuracy score -> 1.0
    max_acceptable_sacc_mps: float = 3.0 # Beyond this, speed accuracy score -> 0
    nominal_sacc_mps: float = 0.2        # Below this, speed accuracy score -> 1.0

    # Satellite count thresholds
    min_satellites: int = 4              # Mathematical minimum for 3D fix
    nominal_satellites: int = 12         # Good constellation view

    # Innovation / Mahalanobis gating
    mahalanobis_gate_trusted: float = 3.0   # 3-sigma gate for trusted measurements
    mahalanobis_gate_degraded: float = 4.5  # Wider gate for degraded measurements
    mahalanobis_gate_hard: float = 6.0      # Absolute rejection threshold

    # Kinematic plausibility limits
    max_plausible_speed_mps: float = 100.0  # 360 km/h (land vehicle upper bound)
    max_plausible_accel_mps2: float = 15.0  # 1.5g max vehicle acceleration
    max_position_jump_rate_mps: float = 50.0  # Max jump distance / dt

    # Covariance scaling parameters
    degraded_cov_scale_min: float = 2.0    # Minimum inflation for degraded
    degraded_cov_scale_max: float = 50.0   # Maximum inflation for degraded
    cov_scale_gamma: float = 2.5           # Exponential penalty factor for low quality

    # Weights for multi-factor quality scoring (must sum to 1.0)
    w_pos_acc: float = 0.30
    w_vel_acc: float = 0.15
    w_sat: float = 0.15
    w_innov: float = 0.25
    w_temp: float = 0.15


class GnssQualityEstimator:
    """Estimates continuous GNSS quality and produces integrity reports.

    Maintains internal history for temporal consistency, jump detection,
    and outlier tracking.
    """

    def __init__(self, config: Optional[GnssIntegrityConfig] = None):
        self.config = config or GnssIntegrityConfig()
        self._prev_fix: Optional[GnssFix] = None
        self._prev_pos_ned: Optional[Tuple[float, float, float]] = None
        self._consecutive_rejected: int = 0
        self._consecutive_trusted: int = 0

    def reset(self) -> None:
        """Reset internal state."""
        self._prev_fix = None
        self._prev_pos_ned = None
        self._consecutive_rejected = 0
        self._consecutive_trusted = 0

    def evaluate(
        self,
        fix: GnssFix,
        current_pos_ned: Optional[Tuple[float, float, float]] = None,
        current_vel_ned: Optional[Tuple[float, float, float]] = None,
        pos_innovation: Optional[np.ndarray] = None,
        pos_innovation_cov: Optional[np.ndarray] = None,
        vel_innovation: Optional[np.ndarray] = None,
        vel_innovation_cov: Optional[np.ndarray] = None,
    ) -> GnssIntegrityReport:
        """Evaluate the integrity of a GNSS fix.

        Args:
            fix: Raw GNSS fix from sensor layer.
            current_pos_ned: Current INS/ESKF position in NED (m).
            current_vel_ned: Current INS/ESKF velocity in NED (m/s).
            pos_innovation: Position measurement residual z = y - h(x) (3,).
            pos_innovation_cov: Innovation covariance S_pos = H P H^T + R (3, 3).
            vel_innovation: Velocity measurement residual (3,).
            vel_innovation_cov: Velocity innovation covariance (3, 3).

        Returns:
            GnssIntegrityReport with quality score, classification, and scaled covariances.
        """
        # 1. Sanity & Numerical Validity Checks (Fail-safe)
        validity_err = self._check_validity(fix)
        if validity_err:
            return self._build_rejected_report(fix, validity_err)

        # 2. Compute individual sub-scores
        pos_acc_score = self._compute_pos_accuracy_score(fix)
        vel_acc_score = self._compute_vel_accuracy_score(fix)
        sat_score = self._compute_satellite_score(fix)
        innov_score, innov_reject = self._compute_innovation_score(
            pos_innovation, pos_innovation_cov,
            vel_innovation, vel_innovation_cov
        )
        temp_score, temp_reject, jump_m, dt_s = self._compute_temporal_score(
            fix, current_pos_ned
        )

        # 3. Check for hard rejections
        if innov_reject:
            return self._build_rejected_report(
                fix, f"Innovation exceeded hard gate: {innov_reject}",
                pos_acc=pos_acc_score, vel_acc=vel_acc_score, sat=sat_score,
                innov=innov_score, temp=temp_score, jump_m=jump_m, dt_s=dt_s
            )

        if temp_reject:
            return self._build_rejected_report(
                fix, f"Kinematic violation: {temp_reject}",
                pos_acc=pos_acc_score, vel_acc=vel_acc_score, sat=sat_score,
                innov=innov_score, temp=temp_score, jump_m=jump_m, dt_s=dt_s
            )

        # 4. Composite Quality Score (weighted linear combination with geometric floor)
        cfg = self.config
        quality = (
            cfg.w_pos_acc * pos_acc_score +
            cfg.w_vel_acc * vel_acc_score +
            cfg.w_sat * sat_score +
            cfg.w_innov * innov_score +
            cfg.w_temp * temp_score
        )
        # Any single near-zero sub-score drags down composite quality
        min_sub = min(pos_acc_score, vel_acc_score, sat_score, innov_score, temp_score)
        quality = float(np.clip(quality * (0.5 + 0.5 * min_sub), 0.0, 1.0))

        # 5. Classification
        # Override: if satellite score is zero (less than 4 satellites), reject
        if sat_score == 0.0:
            classification = GnssClassification.REJECTED
            self._consecutive_rejected += 1
            self._consecutive_trusted = 0
        else:
            # Threshold-based classification
            if quality >= cfg.trusted_threshold:
                classification = GnssClassification.TRUSTED
                self._consecutive_trusted += 1
                self._consecutive_rejected = 0
            elif quality >= cfg.degraded_threshold:
                classification = GnssClassification.DEGRADED
                self._consecutive_trusted = 0
                self._consecutive_rejected = 0
            else:
                classification = GnssClassification.REJECTED
                self._consecutive_rejected += 1
                self._consecutive_trusted = 0

        # 6. Compute Scaled Covariances
        scaled_pos_cov = self._scale_position_covariance(fix, quality, classification)
        scaled_vel_cov = self._scale_velocity_covariance(fix, quality, classification)

        # Update history
        self._prev_fix = fix
        if current_pos_ned is not None:
            self._prev_pos_ned = current_pos_ned

        return GnssIntegrityReport(
            quality_score=quality,
            classification=classification,
            position_accuracy_score=pos_acc_score,
            velocity_accuracy_score=vel_acc_score,
            satellite_score=sat_score,
            innovation_score=innov_score,
            temporal_score=temp_score,
            scaled_pos_cov=scaled_pos_cov,
            scaled_vel_cov=scaled_vel_cov,
            rejection_reason="" if classification != GnssClassification.REJECTED else "Low composite quality",
            position_jump_m=jump_m,
            time_delta_s=dt_s,
        )

    # ------------------------------------------------------------------
    # Sub-score computation functions
    # ------------------------------------------------------------------

    def _check_validity(self, fix: GnssFix) -> Optional[str]:
        """Check for NaN, inf, negative accuracy, out-of-range coords."""
        # NaN / Inf checks
        fields_to_check = [
            fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
            fix.horizontal_accuracy_m, fix.vertical_accuracy_m,
            fix.speed_accuracy_mps,
            fix.velocity_ned_mps[0], fix.velocity_ned_mps[1], fix.velocity_ned_mps[2]
        ]
        for val in fields_to_check:
            if math.isnan(val) or math.isinf(val):
                return "NaN or Inf detected in GNSS fix"

        # Timestamp validity
        if fix.timestamp_ns <= 0:
            return "Invalid timestamp <= 0"
        if self._prev_fix is not None and fix.timestamp_ns <= self._prev_fix.timestamp_ns:
            return f"Non-monotonic timestamp: {fix.timestamp_ns} <= {self._prev_fix.timestamp_ns}"

        # Coordinate bounds
        if not (-90.0 <= fix.latitude_deg <= 90.0):
            return f"Latitude out of bounds: {fix.latitude_deg}"
        if not (-180.0 <= fix.longitude_deg <= 180.0):
            return f"Longitude out of bounds: {fix.longitude_deg}"

        # Accuracy non-negativity
        if fix.horizontal_accuracy_m < 0 or fix.vertical_accuracy_m < 0 or fix.speed_accuracy_mps < 0:
            return "Negative accuracy values reported"

        # Satellite count
        if fix.satellite_count < 0:
            return f"Negative satellite count: {fix.satellite_count}"

        # Speed plausibility
        speed = math.sqrt(sum(v**2 for v in fix.velocity_ned_mps))
        if speed > self.config.max_plausible_speed_mps:
            return f"Implausible velocity norm: {speed:.1f} m/s > {self.config.max_plausible_speed_mps} m/s"

        return None

    def _compute_pos_accuracy_score(self, fix: GnssFix) -> float:
        """Score position accuracy: 1.0 for nominal, smoothly dropping to 0.0 at max."""
        cfg = self.config
        hacc = fix.horizontal_accuracy_m
        vacc = fix.vertical_accuracy_m

        # Sigmoidal drop-off for horizontal accuracy
        if hacc <= cfg.nominal_hacc_m:
            s_h = 1.0
        elif hacc >= cfg.max_acceptable_hacc_m:
            s_h = 0.0
        else:
            ratio = (hacc - cfg.nominal_hacc_m) / (cfg.max_acceptable_hacc_m - cfg.nominal_hacc_m)
            s_h = float(0.5 * (1.0 + math.cos(math.pi * ratio)))

        # Vertical accuracy (typically ~1.5-2x looser than horizontal)
        if vacc <= cfg.nominal_vacc_m:
            s_v = 1.0
        elif vacc >= cfg.max_acceptable_vacc_m:
            s_v = 0.0
        else:
            ratio = (vacc - cfg.nominal_vacc_m) / (cfg.max_acceptable_vacc_m - cfg.nominal_vacc_m)
            s_v = float(0.5 * (1.0 + math.cos(math.pi * ratio)))

        return float(np.clip(0.7 * s_h + 0.3 * s_v, 0.0, 1.0))

    def _compute_vel_accuracy_score(self, fix: GnssFix) -> float:
        """Score speed accuracy: 1.0 for nominal, smoothly dropping to 0.0 at max."""
        cfg = self.config
        sacc = fix.speed_accuracy_mps

        if sacc <= cfg.nominal_sacc_mps:
            return 1.0
        elif sacc >= cfg.max_acceptable_sacc_mps:
            return 0.0
        else:
            ratio = (sacc - cfg.nominal_sacc_mps) / (cfg.max_acceptable_sacc_mps - cfg.nominal_sacc_mps)
            return float(np.clip(0.5 * (1.0 + math.cos(math.pi * ratio)), 0.0, 1.0))

    def _compute_satellite_score(self, fix: GnssFix) -> float:
        """Score satellite count: 0 for < 4 sats, linear ramp to 1.0 at nominal (12+)."""
        cfg = self.config
        sats = fix.satellite_count

        if sats < cfg.min_satellites:
            return 0.0
        if sats >= cfg.nominal_satellites:
            return 1.0

        return float((sats - cfg.min_satellites) / (cfg.nominal_satellites - cfg.min_satellites))

    def _compute_innovation_score(
        self,
        pos_innov: Optional[np.ndarray],
        pos_cov: Optional[np.ndarray],
        vel_innov: Optional[np.ndarray],
        vel_cov: Optional[np.ndarray],
    ) -> Tuple[float, Optional[str]]:
        """Compute innovation consistency score and check for hard gate rejection.

        Returns:
            (score [0, 1], rejection_reason or None)
        """
        cfg = self.config
        scores = []

        if pos_innov is not None and pos_cov is not None:
            try:
                S_inv = np.linalg.pinv(pos_cov) if np.linalg.cond(pos_cov) > 1e10 else np.linalg.inv(pos_cov)
                d2 = float(pos_innov.T @ S_inv @ pos_innov)
                d = math.sqrt(max(0.0, d2))

                if d > cfg.mahalanobis_gate_hard:
                    return 0.0, f"Position Mahalanobis {d:.2f} > hard gate {cfg.mahalanobis_gate_hard}"

                # Score drops smoothly from 1.0 (d=0) to 0.0 at hard gate
                if d <= cfg.mahalanobis_gate_trusted:
                    scores.append(1.0 - 0.3 * (d / cfg.mahalanobis_gate_trusted))
                else:
                    ratio = (d - cfg.mahalanobis_gate_trusted) / (cfg.mahalanobis_gate_hard - cfg.mahalanobis_gate_trusted)
                    scores.append(max(0.0, 0.7 * (1.0 - ratio)))
            except np.linalg.LinAlgError:
                scores.append(0.5)

        if vel_innov is not None and vel_cov is not None:
            try:
                S_inv = np.linalg.pinv(vel_cov) if np.linalg.cond(vel_cov) > 1e10 else np.linalg.inv(vel_cov)
                d2 = float(vel_innov.T @ S_inv @ vel_innov)
                d = math.sqrt(max(0.0, d2))

                if d > cfg.mahalanobis_gate_hard:
                    return 0.0, f"Velocity Mahalanobis {d:.2f} > hard gate {cfg.mahalanobis_gate_hard}"

                if d <= cfg.mahalanobis_gate_trusted:
                    scores.append(1.0 - 0.3 * (d / cfg.mahalanobis_gate_trusted))
                else:
                    ratio = (d - cfg.mahalanobis_gate_trusted) / (cfg.mahalanobis_gate_hard - cfg.mahalanobis_gate_trusted)
                    scores.append(max(0.0, 0.7 * (1.0 - ratio)))
            except np.linalg.LinAlgError:
                scores.append(0.5)

        if not scores:
            return 1.0, None  # No innovation info provided, neutral trust

        return float(np.clip(np.mean(scores), 0.0, 1.0)), None

    def _compute_temporal_score(
        self,
        fix: GnssFix,
        current_pos_ned: Optional[Tuple[float, float, float]],
    ) -> Tuple[float, Optional[str], Optional[float], Optional[float]]:
        """Check temporal consistency: position jump rate, acceleration plausibility.

        Returns:
            (score [0, 1], rejection_reason or None, jump_distance_m, dt_seconds)
        """
        if self._prev_fix is None:
            return 1.0, None, None, None

        dt_s = (fix.timestamp_ns - self._prev_fix.timestamp_ns) * 1e-9
        if dt_s <= 0.0:
            return 0.0, f"Non-positive dt: {dt_s:.4f}s", None, dt_s

        # Estimate displacement using Haversine / flat-Earth approximation
        d_lat_m = (fix.latitude_deg - self._prev_fix.latitude_deg) * 111139.0
        mean_lat_rad = math.radians(0.5 * (fix.latitude_deg + self._prev_fix.latitude_deg))
        d_lon_m = (fix.longitude_deg - self._prev_fix.longitude_deg) * 111139.0 * math.cos(mean_lat_rad)
        d_alt_m = fix.altitude_m - self._prev_fix.altitude_m
        jump_m = math.sqrt(d_lat_m**2 + d_lon_m**2 + d_alt_m**2)

        implied_speed = jump_m / dt_s

        # Hard rejection on impossible jump speed
        if implied_speed > self.config.max_position_jump_rate_mps:
            return (
                0.0,
                f"Position jump rate {implied_speed:.1f} m/s > max {self.config.max_position_jump_rate_mps} m/s",
                jump_m,
                dt_s
            )

        # Acceleration plausibility check between fixes
        v_prev = np.array(self._prev_fix.velocity_ned_mps)
        v_curr = np.array(fix.velocity_ned_mps)
        accel_apparent = float(np.linalg.norm(v_curr - v_prev) / dt_s)

        if accel_apparent > self.config.max_plausible_accel_mps2 * 2.0:
            return (
                0.0,
                f"Implausible acceleration between fixes: {accel_apparent:.1f} m/s²",
                jump_m,
                dt_s
            )

        # Score based on kinematic smoothness
        # Speed ratio against plausible max
        speed_ratio = implied_speed / self.config.max_position_jump_rate_mps
        accel_ratio = accel_apparent / (self.config.max_plausible_accel_mps2 * 2.0)

        s_kin = max(0.0, 1.0 - max(speed_ratio, accel_ratio))
        return float(np.clip(s_kin, 0.0, 1.0)), None, jump_m, dt_s

    # ------------------------------------------------------------------
    # Covariance Scaling
    # ------------------------------------------------------------------

    def _scale_position_covariance(
        self,
        fix: GnssFix,
        quality: float,
        classification: GnssClassification,
    ) -> np.ndarray:
        """Compute 3x3 position measurement covariance R_pos scaled by quality.

        R_pos = diag(sigma_h^2, sigma_h^2, sigma_v^2) * scale_factor(quality)
        """
        cfg = self.config
        var_h = max(0.01, fix.horizontal_accuracy_m ** 2)
        var_v = max(0.04, fix.vertical_accuracy_m ** 2)
        R_base = np.diag([var_h, var_h, var_v])

        if classification == GnssClassification.TRUSTED:
            # Trusted: slight inflation for quality < 1.0
            scale = 1.0 + (1.0 - quality) * 1.5
            return R_base * scale

        elif classification == GnssClassification.DEGRADED:
            # Degraded: power-law scaling
            # As quality goes from degraded_threshold to 0, scale increases exponentially
            q_norm = (quality - cfg.degraded_threshold) / (cfg.trusted_threshold - cfg.degraded_threshold)
            q_norm = float(np.clip(q_norm, 0.0, 1.0))
            scale = cfg.degraded_cov_scale_min + (1.0 - q_norm)**cfg.cov_scale_gamma * (
                cfg.degraded_cov_scale_max - cfg.degraded_cov_scale_min
            )
            return R_base * scale

        else:
            # REJECTED: return huge covariance
            return R_base * 1e8

    def _scale_velocity_covariance(
        self,
        fix: GnssFix,
        quality: float,
        classification: GnssClassification,
    ) -> np.ndarray:
        """Compute 3x3 velocity measurement covariance R_vel scaled by quality."""
        cfg = self.config
        var_s = max(0.0025, fix.speed_accuracy_mps ** 2)
        # Vertical velocity typically less accurate
        R_base = np.diag([var_s, var_s, var_s * 2.0])

        if classification == GnssClassification.TRUSTED:
            scale = 1.0 + (1.0 - quality) * 1.5
            return R_base * scale
        elif classification == GnssClassification.DEGRADED:
            q_norm = (quality - cfg.degraded_threshold) / (cfg.trusted_threshold - cfg.degraded_threshold)
            q_norm = float(np.clip(q_norm, 0.0, 1.0))
            scale = cfg.degraded_cov_scale_min + (1.0 - q_norm)**cfg.cov_scale_gamma * (
                cfg.degraded_cov_scale_max - cfg.degraded_cov_scale_min
            )
            return R_base * scale
        else:
            return R_base * 1e8

    def _build_rejected_report(
        self,
        fix: GnssFix,
        reason: str,
        pos_acc: float = 0.0,
        vel_acc: float = 0.0,
        sat: float = 0.0,
        innov: float = 0.0,
        temp: float = 0.0,
        jump_m: Optional[float] = None,
        dt_s: Optional[float] = None,
    ) -> GnssIntegrityReport:
        """Build an integrity report for a rejected measurement."""
        var_h = max(0.01, fix.horizontal_accuracy_m ** 2) if not math.isnan(fix.horizontal_accuracy_m) else 100.0
        var_v = max(0.04, fix.vertical_accuracy_m ** 2) if not math.isnan(fix.vertical_accuracy_m) else 100.0
        var_s = max(0.0025, fix.speed_accuracy_mps ** 2) if not math.isnan(fix.speed_accuracy_mps) else 10.0

        return GnssIntegrityReport(
            quality_score=0.0,
            classification=GnssClassification.REJECTED,
            position_accuracy_score=pos_acc,
            velocity_accuracy_score=vel_acc,
            satellite_score=sat,
            innovation_score=innov,
            temporal_score=temp,
            scaled_pos_cov=np.diag([var_h, var_h, var_v]) * 1e8,
            scaled_vel_cov=np.diag([var_s, var_s, var_s * 2.0]) * 1e8,
            rejection_reason=reason,
            position_jump_m=jump_m,
            time_delta_s=dt_s,
        )
