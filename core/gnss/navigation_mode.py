"""
Navigation Mode State Machine for Seamless GNSS <-> Dead Reckoning Transitions.

Enforces strict state transitions with hysteresis, persistence counters,
outage tracking, and smooth re-acquisition logic to prevent discontinuous jumps.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List, Dict, Any
import numpy as np

from core.gnss.integrity import GnssClassification, GnssIntegrityReport


class NavigationMode(Enum):
    """Navigation system operational modes."""
    GNSS_FIXED = auto()         # Full GNSS availability and high confidence
    HYBRID_DEGRADED = auto()    # Degraded GNSS, fused with inflated uncertainty
    TRANSITION_TO_DR = auto()   # GNSS newly lost; latching biases, configuring DR
    DEAD_RECKONING = auto()     # Pure inertial + motion intelligence (ZUPT / ML vel)
    TRANSITION_TO_GNSS = auto() # GNSS newly recovered; validating consistency
    RECOVERED = auto()          # Verified recovery, smooth gain ramping active


@dataclass
class ModeTransitionEvent:
    """Record of a navigation mode transition."""
    from_mode: NavigationMode
    to_mode: NavigationMode
    timestamp_ns: int
    trigger_reason: str
    quality_score: float
    outage_duration_s: float = 0.0


@dataclass
class NavigationModeConfig:
    """Configuration for mode transitions and hysteresis."""
    # Persistence counters for transitions (number of consecutive cycles)
    recovery_persistence_count: int = 3   # Consecutive TRUSTED fixes required to exit DR
    outage_persistence_count: int = 2     # Consecutive REJECTED/missing fixes to declare outage
    degraded_persistence_count: int = 2   # Consecutive DEGRADED fixes to switch to HYBRID

    # Outage duration thresholds (seconds)
    gnss_timeout_s: float = 2.0           # Time without GNSS fix before assuming outage
    long_outage_threshold_s: float = 30.0 # Threshold for long outage (triggers higher recovery caution)

    # Smooth gain ramping parameters for recovery
    recovery_ramp_steps: int = 5          # Number of update steps to ramp measurement weight from 0 -> 1
    recovery_innovation_gate: float = 3.5 # Strict gate during re-acquisition

    # Process noise inflation factors during DR
    q_pos_inflation_dr: float = 2.0       # Multiply position process noise during DR
    q_vel_inflation_dr: float = 1.5       # Multiply velocity process noise during DR
    q_bias_inflation_dr: float = 0.5      # Reduce bias process noise during DR (latch biases)


class NavigationModeManager:
    """State machine governing navigation modes and transitions.

    Ensures that transitions between GNSS-aided and Dead Reckoning modes
    are smooth, stable, and protected against transient drops or outliers.
    """

    def __init__(self, config: Optional[NavigationModeConfig] = None):
        self.config = config or NavigationModeConfig()
        self._current_mode: NavigationMode = NavigationMode.GNSS_FIXED
        self._prev_mode: NavigationMode = NavigationMode.GNSS_FIXED

        # Persistence counters
        self._consecutive_trusted: int = 0
        self._consecutive_degraded: int = 0
        self._consecutive_rejected: int = 0
        self._consecutive_missing: int = 0

        # Outage and recovery tracking
        self._last_trusted_gnss_ts_ns: Optional[int] = None
        self._outage_start_ts_ns: Optional[int] = None
        self._recovery_start_ts_ns: Optional[int] = None
        self._recovery_step_count: int = 0
        self._total_outage_duration_s: float = 0.0

        # Latched states at outage onset
        self._latched_accel_bias: Optional[Tuple[float, float, float]] = None
        self._latched_gyro_bias: Optional[Tuple[float, float, float]] = None

        # Transition history
        self._transition_history: List[ModeTransitionEvent] = []

    @property
    def current_mode(self) -> NavigationMode:
        """Current operational mode."""
        return self._current_mode

    @property
    def is_in_outage(self) -> bool:
        """True if in TRANSITION_TO_DR, DEAD_RECKONING, or TRANSITION_TO_GNSS."""
        return self._current_mode in (
            NavigationMode.TRANSITION_TO_DR,
            NavigationMode.DEAD_RECKONING,
            NavigationMode.TRANSITION_TO_GNSS
        )

    @property
    def recovery_gain_scale(self) -> float:
        """Gain multiplier [0.0, 1.0] for smooth measurement ramping during recovery.

        Prevents position/velocity jumps upon re-acquiring GNSS.
        """
        if self._current_mode == NavigationMode.TRANSITION_TO_GNSS:
            # During validation, apply minimal/zero gain
            return 0.1
        elif self._current_mode == NavigationMode.RECOVERED:
            # Linear ramp from 0.2 to 1.0
            fraction = min(1.0, (self._recovery_step_count + 1) / self.config.recovery_ramp_steps)
            return float(0.2 + 0.8 * fraction)
        elif self._current_mode == NavigationMode.HYBRID_DEGRADED:
            return 0.5
        else:
            return 1.0

    @property
    def transition_history(self) -> List[ModeTransitionEvent]:
        """List of all mode transition events."""
        return list(self._transition_history)

    def reset(self) -> None:
        """Reset state machine to initial conditions."""
        self._current_mode = NavigationMode.GNSS_FIXED
        self._prev_mode = NavigationMode.GNSS_FIXED
        self._consecutive_trusted = 0
        self._consecutive_degraded = 0
        self._consecutive_rejected = 0
        self._consecutive_missing = 0
        self._last_trusted_gnss_ts_ns = None
        self._outage_start_ts_ns = None
        self._recovery_start_ts_ns = None
        self._recovery_step_count = 0
        self._total_outage_duration_s = 0.0
        self._latched_accel_bias = None
        self._latched_gyro_bias = None
        self._transition_history.clear()

    def update_with_gnss(
        self,
        report: GnssIntegrityReport,
        timestamp_ns: int,
        accel_bias: Optional[Tuple[float, float, float]] = None,
        gyro_bias: Optional[Tuple[float, float, float]] = None,
    ) -> NavigationMode:
        """Update navigation mode based on new GNSS integrity report.

        Args:
            report: GNSS integrity evaluation report.
            timestamp_ns: Current timestamp in nanoseconds.
            accel_bias: Current estimated accel bias (to latch if outage begins).
            gyro_bias: Current estimated gyro bias (to latch if outage begins).

        Returns:
            Updated NavigationMode.
        """
        classification = report.classification
        quality = report.quality_score

        # Update persistence counters
        if classification == GnssClassification.TRUSTED:
            self._consecutive_trusted += 1
            self._consecutive_degraded = 0
            self._consecutive_rejected = 0
            self._consecutive_missing = 0
            self._last_trusted_gnss_ts_ns = timestamp_ns
        elif classification == GnssClassification.DEGRADED:
            self._consecutive_degraded += 1
            self._consecutive_trusted = 0
            self._consecutive_rejected = 0
            self._consecutive_missing = 0
        else:  # REJECTED
            self._consecutive_rejected += 1
            self._consecutive_trusted = 0
            self._consecutive_degraded = 0

        # Execute State Machine Transitions
        new_mode = self._evaluate_mode_transition(timestamp_ns, quality, accel_bias, gyro_bias)
        return new_mode

    def update_without_gnss(
        self,
        timestamp_ns: int,
        accel_bias: Optional[Tuple[float, float, float]] = None,
        gyro_bias: Optional[Tuple[float, float, float]] = None,
    ) -> NavigationMode:
        """Update mode when no GNSS measurement is received (e.g. at IMU rate).

        Args:
            timestamp_ns: Current timestamp in nanoseconds.
            accel_bias: Current accel bias.
            gyro_bias: Current gyro bias.

        Returns:
            Updated NavigationMode.
        """
        self._consecutive_missing += 1
        self._consecutive_trusted = 0

        # Check for timeout if we were in GNSS or HYBRID mode
        if self._last_trusted_gnss_ts_ns is not None:
            time_since_gnss_s = (timestamp_ns - self._last_trusted_gnss_ts_ns) * 1e-9
            if time_since_gnss_s > self.config.gnss_timeout_s:
                if self._current_mode in (NavigationMode.GNSS_FIXED, NavigationMode.HYBRID_DEGRADED):
                    self._transition_to(
                        NavigationMode.TRANSITION_TO_DR,
                        timestamp_ns,
                        f"GNSS timeout ({time_since_gnss_s:.2f}s > {self.config.gnss_timeout_s}s)",
                        0.0,
                        accel_bias,
                        gyro_bias,
                    )
                elif self._current_mode == NavigationMode.TRANSITION_TO_DR:
                    # Advance to steady-state DR
                    self._transition_to(
                        NavigationMode.DEAD_RECKONING,
                        timestamp_ns,
                        "Steady-state DR active",
                        0.0,
                    )

        return self._current_mode

    def step_recovery(self) -> None:
        """Increment recovery step counter during RECOVERED mode."""
        if self._current_mode == NavigationMode.RECOVERED:
            self._recovery_step_count += 1
            if self._recovery_step_count >= self.config.recovery_ramp_steps:
                # Ramp complete, return to GNSS_FIXED
                self._transition_to(
                    NavigationMode.GNSS_FIXED,
                    self._recovery_start_ts_ns or 0,
                    "Recovery ramp complete -> GNSS_FIXED",
                    1.0,
                )

    # ------------------------------------------------------------------
    # Internal State Machine Logic
    # ------------------------------------------------------------------

    def _evaluate_mode_transition(
        self,
        timestamp_ns: int,
        quality: float,
        accel_bias: Optional[Tuple[float, float, float]],
        gyro_bias: Optional[Tuple[float, float, float]],
    ) -> NavigationMode:
        cfg = self.config
        mode = self._current_mode

        # ----------------------------------------------------------------
        # 1. From GNSS_FIXED
        # ----------------------------------------------------------------
        if mode == NavigationMode.GNSS_FIXED:
            if self._consecutive_rejected >= cfg.outage_persistence_count:
                self._transition_to(
                    NavigationMode.TRANSITION_TO_DR,
                    timestamp_ns,
                    f"Outage detected: {self._consecutive_rejected} rejected fixes",
                    quality,
                    accel_bias,
                    gyro_bias,
                )
            elif self._consecutive_degraded >= cfg.degraded_persistence_count:
                self._transition_to(
                    NavigationMode.HYBRID_DEGRADED,
                    timestamp_ns,
                    f"Degraded GNSS: {self._consecutive_degraded} degraded fixes",
                    quality,
                )

        # ----------------------------------------------------------------
        # 2. From HYBRID_DEGRADED
        # ----------------------------------------------------------------
        elif mode == NavigationMode.HYBRID_DEGRADED:
            if self._consecutive_trusted >= cfg.recovery_persistence_count:
                self._transition_to(
                    NavigationMode.GNSS_FIXED,
                    timestamp_ns,
                    f"GNSS quality restored ({self._consecutive_trusted} trusted fixes)",
                    quality,
                )
            elif self._consecutive_rejected >= cfg.outage_persistence_count:
                self._transition_to(
                    NavigationMode.TRANSITION_TO_DR,
                    timestamp_ns,
                    "Degraded GNSS degraded further to complete outage",
                    quality,
                    accel_bias,
                    gyro_bias,
                )

        # ----------------------------------------------------------------
        # 3. From TRANSITION_TO_DR
        # ----------------------------------------------------------------
        elif mode == NavigationMode.TRANSITION_TO_DR:
            # Check if GNSS immediately returned (e.g. transient glitch)
            if self._consecutive_trusted >= cfg.recovery_persistence_count:
                self._transition_to(
                    NavigationMode.TRANSITION_TO_GNSS,
                    timestamp_ns,
                    "Early GNSS recovery from DR transition",
                    quality,
                )
            else:
                # Advance to steady-state DR
                self._transition_to(
                    NavigationMode.DEAD_RECKONING,
                    timestamp_ns,
                    "Steady-state DR active",
                    quality,
                )

        # ----------------------------------------------------------------
        # 4. From DEAD_RECKONING
        # ----------------------------------------------------------------
        elif mode == NavigationMode.DEAD_RECKONING:
            # Check for GNSS recovery: requires multiple consecutive TRUSTED fixes
            if self._consecutive_trusted >= cfg.recovery_persistence_count:
                self._transition_to(
                    NavigationMode.TRANSITION_TO_GNSS,
                    timestamp_ns,
                    f"GNSS reappeared: {self._consecutive_trusted} consecutive trusted fixes",
                    quality,
                )

        # ----------------------------------------------------------------
        # 5. From TRANSITION_TO_GNSS
        # ----------------------------------------------------------------
        elif mode == NavigationMode.TRANSITION_TO_GNSS:
            # If GNSS remains TRUSTED, proceed to RECOVERED (ramp)
            if self._consecutive_trusted >= cfg.recovery_persistence_count + 1:
                self._transition_to(
                    NavigationMode.RECOVERED,
                    timestamp_ns,
                    "GNSS stability confirmed, starting smooth gain ramp",
                    quality,
                )
            elif self._consecutive_rejected > 0:
                # Recovery was a false alarm, return to DR
                self._transition_to(
                    NavigationMode.DEAD_RECKONING,
                    timestamp_ns,
                    "Recovery rejected by inconsistent fix, returning to DR",
                    quality,
                )

        # ----------------------------------------------------------------
        # 6. From RECOVERED
        # ----------------------------------------------------------------
        elif mode == NavigationMode.RECOVERED:
            if self._consecutive_rejected >= cfg.outage_persistence_count:
                # Intermittent drop during recovery
                self._transition_to(
                    NavigationMode.DEAD_RECKONING,
                    timestamp_ns,
                    "Outage recurred during recovery ramp",
                    quality,
                )
            # Ramping handled by step_recovery()

        return self._current_mode

    def _transition_to(
        self,
        target_mode: NavigationMode,
        timestamp_ns: int,
        reason: str,
        quality: float,
        accel_bias: Optional[Tuple[float, float, float]] = None,
        gyro_bias: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        """Execute and record a mode transition."""
        if target_mode == self._current_mode:
            return

        # Record outage start/end
        outage_duration = 0.0
        if target_mode == NavigationMode.TRANSITION_TO_DR:
            self._outage_start_ts_ns = timestamp_ns
            if accel_bias is not None:
                self._latched_accel_bias = accel_bias
            if gyro_bias is not None:
                self._latched_gyro_bias = gyro_bias
        elif target_mode == NavigationMode.TRANSITION_TO_GNSS:
            if self._outage_start_ts_ns is not None:
                outage_duration = (timestamp_ns - self._outage_start_ts_ns) * 1e-9
                self._total_outage_duration_s = outage_duration
            self._recovery_start_ts_ns = timestamp_ns
            self._recovery_step_count = 0

        event = ModeTransitionEvent(
            from_mode=self._current_mode,
            to_mode=target_mode,
            timestamp_ns=timestamp_ns,
            trigger_reason=reason,
            quality_score=quality,
            outage_duration_s=outage_duration,
        )
        self._transition_history.append(event)
        self._prev_mode = self._current_mode
        self._current_mode = target_mode
