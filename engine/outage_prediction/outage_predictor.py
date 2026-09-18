"""
engine/outage_prediction/outage_predictor.py

Predictive Outage Detection (Phase 8).
Maintains a short trailing trend of Android raw GNSS measurements (C/N0, satellite count, accuracy).
Detects degradation trends (tunnel approaches, urban canyons) and emits a continuous trust signal [0.0, 1.0].
"""

import numpy as np
from collections import deque
from typing import Optional, List, Tuple

class OutagePredictor:
    def __init__(self, window_size_sec: float = 3.0, dt: float = 1.0):
        """
        :param window_size_sec: Duration of trailing window to monitor trends.
        :param dt: GNSS update interval (usually 1.0s for consumer phones).
        """
        self.window_size_sec = window_size_sec
        self.dt = dt
        self.max_len = max(3, int(window_size_sec / dt))

        # Ring buffers for trend calculation
        self.cn0_buffer = deque(maxlen=self.max_len)
        self.sat_count_buffer = deque(maxlen=self.max_len)
        self.acc_buffer = deque(maxlen=self.max_len)

        self.current_trust: float = 1.0  # 1.0 = fully trusted, 0.0 = severe degradation

        # Thresholds (calibrated for standard smartphone GNSS)
        self.strong_cn0 = 35.0          # dB-Hz
        self.weak_cn0 = 20.0            # dB-Hz
        self.good_sat_count = 12
        self.poor_sat_count = 4

    def _calculate_trend(self, buffer: deque, is_inverted: bool = False) -> float:
        """
        Calculate normalized trend over the buffer.
        Returns a derivative-like score.
        is_inverted = True (e.g. for accuracy where higher is worse).
        """
        if len(buffer) < 2:
            return 0.0

        y = np.array(buffer)
        x = np.arange(len(y))

        # Fit straight line
        slope, _ = np.polyfit(x, y, 1)

        if is_inverted:
            return -slope
        return slope

    def update(
        self,
        avg_cn0: Optional[float] = None,
        sat_count: Optional[int] = None,
        accuracy_m: Optional[float] = None
    ) -> float:
        """
        Process a new epoch of GNSS status data.
        Returns the updated trust signal [0.0, 1.0].
        """
        if avg_cn0 is not None:
            self.cn0_buffer.append(avg_cn0)

        if sat_count is not None:
            self.sat_count_buffer.append(sat_count)

        if accuracy_m is not None:
            self.acc_buffer.append(accuracy_m)

        # If we have insufficient history, assume high trust initially but decay if current values are terrible
        if len(self.cn0_buffer) < 2 and len(self.sat_count_buffer) < 2:
            self.current_trust = 1.0
            if sat_count is not None and sat_count <= self.poor_sat_count:
                self.current_trust = 0.1
            return self.current_trust

        # 1. Absolute level scoring
        trust_levels = []

        if len(self.cn0_buffer) > 0:
            cn0 = self.cn0_buffer[-1]
            t_cn0 = np.clip((cn0 - self.weak_cn0) / (self.strong_cn0 - self.weak_cn0), 0.0, 1.0)
            trust_levels.append(t_cn0)

        if len(self.sat_count_buffer) > 0:
            sats = self.sat_count_buffer[-1]
            t_sats = np.clip((sats - self.poor_sat_count) / max(1, self.good_sat_count - self.poor_sat_count), 0.0, 1.0)
            trust_levels.append(t_sats)

        base_trust = np.mean(trust_levels) if trust_levels else 1.0

        # 2. Trend (Derivative) scoring - looking for sudden sharp drops
        trend_penalty = 0.0

        # Sudden drop in satellites? Use negative slope
        if len(self.sat_count_buffer) >= 2:
            sat_trend = self._calculate_trend(self.sat_count_buffer)
            if sat_trend < -1.0:  # Dropping more than 1 sat per epoch on average
                trend_penalty += np.clip(abs(sat_trend) * 0.1, 0.0, 0.3)

        # Sudden drop in signal strength?
        if len(self.cn0_buffer) >= 2:
            cn0_trend = self._calculate_trend(self.cn0_buffer)
            if cn0_trend < -2.0:  # Dropping 2 dB-Hz per epoch
                trend_penalty += np.clip(abs(cn0_trend) * 0.05, 0.0, 0.2)

        # Combine
        self.current_trust = np.clip(base_trust - trend_penalty, 0.0, 1.0)
        return self.current_trust

    def reset(self):
        """Clear buffers on a hard reset."""
        self.cn0_buffer.clear()
        self.sat_count_buffer.clear()
        self.acc_buffer.clear()
        self.current_trust = 1.0
