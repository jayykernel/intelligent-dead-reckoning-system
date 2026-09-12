"""
GNSS Outage and Integrity Manager.

Integrates GNSS Quality Estimation, Navigation Mode State Machine,
and ESKF measurement updates to provide seamless transitions between
GNSS-assisted navigation and Dead Reckoning.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
import numpy as np

from core.sensors.data_types import GnssFix
from core.filters.eskf import ErrorStateKalmanFilter, UpdateResult
from core.navigation.earth import lla_to_ned
from core.gnss.integrity import (
    GnssQualityEstimator,
    GnssIntegrityReport,
    GnssClassification,
    GnssIntegrityConfig,
)
from core.gnss.navigation_mode import (
    NavigationModeManager,
    NavigationMode,
    NavigationModeConfig,
    ModeTransitionEvent,
)


@dataclass
class GnssProcessingResult:
    """Result of processing a GNSS fix through the integrity pipeline.

    Attributes:
        accepted_by_filter: Whether measurement was applied to ESKF.
        navigation_mode: Operational navigation mode after processing.
        integrity_report: Complete GNSS integrity report.
        pos_update_result: ESKF position update result (if attempted).
        vel_update_result: ESKF velocity update result (if attempted).
        mode_transitioned: True if this fix caused a mode transition.
    """
    accepted_by_filter: bool
    navigation_mode: NavigationMode
    integrity_report: GnssIntegrityReport
    pos_update_result: Optional[UpdateResult] = None
    vel_update_result: Optional[UpdateResult] = None
    mode_transitioned: bool = False


class GnssIntegrityPipeline:
    """End-to-end GNSS integrity and transition controller.

    Coordinates:
    - Quality scoring & outlier rejection
    - Dynamic covariance inflation
    - Navigation mode transitions
    - Smooth recovery gain ramping
    - Process noise modulation during outages
    """

    def __init__(
        self,
        eskf: ErrorStateKalmanFilter,
        origin_lla: Optional[Tuple[float, float, float]] = None,
        integrity_config: Optional[GnssIntegrityConfig] = None,
        mode_config: Optional[NavigationModeConfig] = None,
    ):
        self.eskf = eskf
        self.origin_lla = origin_lla
        self.quality_estimator = GnssQualityEstimator(integrity_config)
        self.mode_manager = NavigationModeManager(mode_config)

        # Baseline process noise saved for restoration
        self._nominal_q_diag = np.diag(self.eskf.ins.covariance).copy()

    def set_origin(self, origin_lla: Tuple[float, float, float]) -> None:
        """Set or update WGS-84 origin reference."""
        self.origin_lla = origin_lla

    def reset(self) -> None:
        """Reset internal pipeline states."""
        self.quality_estimator.reset()
        self.mode_manager.reset()

    def process_gnss_fix(self, fix: GnssFix) -> GnssProcessingResult:
        """Process an incoming GNSS fix with full integrity assessment.

        Steps:
        1. Convert LLA to NED (if origin set)
        2. Compute preliminary innovations & innovation covariances
        3. Evaluate GNSS integrity & classification
        4. Update navigation mode state machine
        5. Apply scaled measurement to ESKF (if trusted/degraded)
        6. Manage recovery ramping

        Args:
            fix: Raw GNSS fix.

        Returns:
            GnssProcessingResult containing filter update status and mode.
        """
        # Ensure origin is initialized
        if self.origin_lla is None:
            self.origin_lla = (fix.latitude_deg, fix.longitude_deg, fix.altitude_m)

        # 1. Transform LLA to NED
        pos_ned = lla_to_ned(
            fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
            self.origin_lla[0], self.origin_lla[1], self.origin_lla[2]
        )

        current_pos_ned = self.eskf.ins.state.position_m
        current_vel_ned = self.eskf.ins.state.velocity_mps

        # 2. Compute preliminary innovations for integrity evaluation
        z_pos = np.array(pos_ned) - np.array(current_pos_ned)
        H_pos = np.zeros((3, 15))
        H_pos[0:3, 0:3] = np.eye(3)
        R_pos_raw = np.diag([
            max(0.01, fix.horizontal_accuracy_m ** 2),
            max(0.01, fix.horizontal_accuracy_m ** 2),
            max(0.04, fix.vertical_accuracy_m ** 2)
        ])
        S_pos = H_pos @ self.eskf.ins.covariance @ H_pos.T + R_pos_raw

        z_vel = np.array(fix.velocity_ned_mps) - np.array(current_vel_ned)
        H_vel = np.zeros((3, 15))
        H_vel[0:3, 3:6] = np.eye(3)
        R_vel_raw = np.diag([
            max(0.0025, fix.speed_accuracy_mps ** 2),
            max(0.0025, fix.speed_accuracy_mps ** 2),
            max(0.01, fix.speed_accuracy_mps ** 2 * 2.0)
        ])
        S_vel = H_vel @ self.eskf.ins.covariance @ H_vel.T + R_vel_raw

        # 3. Evaluate Integrity
        report = self.quality_estimator.evaluate(
            fix=fix,
            current_pos_ned=current_pos_ned,
            current_vel_ned=current_vel_ned,
            pos_innovation=z_pos,
            pos_innovation_cov=S_pos,
            vel_innovation=z_vel,
            vel_innovation_cov=S_vel,
        )

        # 4. Update Navigation Mode
        prev_mode = self.mode_manager.current_mode
        state = self.eskf.ins.state
        new_mode = self.mode_manager.update_with_gnss(
            report=report,
            timestamp_ns=fix.timestamp_ns,
            accel_bias=state.accel_bias_mps2,
            gyro_bias=state.gyro_bias_radps,
        )
        mode_changed = (new_mode != prev_mode)

        # 5. Apply to ESKF based on classification and mode
        pos_res: Optional[UpdateResult] = None
        vel_res: Optional[UpdateResult] = None
        applied = False

        if report.classification != GnssClassification.REJECTED and new_mode != NavigationMode.DEAD_RECKONING:
            # Apply recovery gain scaling to covariance (inflation = 1 / gain_scale)
            gain_scale = self.mode_manager.recovery_gain_scale
            inv_gain = max(1.0, 1.0 / max(0.01, gain_scale))

            eff_pos_cov = report.scaled_pos_cov * inv_gain
            eff_vel_cov = report.scaled_vel_cov * inv_gain

            # Adaptive gate based on mode
            gate = (
                self.mode_manager.config.recovery_innovation_gate
                if new_mode in (NavigationMode.TRANSITION_TO_GNSS, NavigationMode.RECOVERED)
                else 4.5
            )

            pos_res = self.eskf.update_position(pos_ned, eff_pos_cov, gate=gate)
            vel_res = self.eskf.update_velocity(fix.velocity_ned_mps, eff_vel_cov, gate=gate)

            applied = bool(pos_res.accepted and vel_res.accepted)

            # If during RECOVERED mode, step the recovery counter
            if new_mode == NavigationMode.RECOVERED and applied:
                self.mode_manager.step_recovery()

        return GnssProcessingResult(
            accepted_by_filter=applied,
            navigation_mode=new_mode,
            integrity_report=report,
            pos_update_result=pos_res,
            vel_update_result=vel_res,
            mode_transitioned=mode_changed,
        )

    def process_imu_tick(self, timestamp_ns: int) -> NavigationMode:
        """Called at IMU rate when no GNSS is available.

        Monitors GNSS timeouts and maintains outage transitions.

        Args:
            timestamp_ns: Current IMU timestamp.

        Returns:
            Current NavigationMode.
        """
        state = self.eskf.ins.state
        return self.mode_manager.update_without_gnss(
            timestamp_ns=timestamp_ns,
            accel_bias=state.accel_bias_mps2,
            gyro_bias=state.gyro_bias_radps,
        )
