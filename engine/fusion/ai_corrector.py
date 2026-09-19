"""
engine/fusion/ai_corrector.py

AI Correction Module (N7 - MEMS / Mobile Path).

Integrates the TFLite AI Speed & Vibration Filter into the GNSS+INS Fusion framework.
During GNSS outages or low-confidence periods, injects learned velocity estimates
as pseudo-measurements and dynamically adapts EKF process noise based on road/engine vibration.
"""

import os
import numpy as np
from typing import Dict, Tuple, Optional, List

from engine.ai_filters.speed_filter import SpeedFilter


class AICorrectionModule:
    def __init__(
        self,
        model_path: str = "training/models/speed_filter.tflite",
        window_size: int = 10,
        base_sigma_ai: float = 1.0,
        enable_tflite: bool = True
    ):
        self.model_path = model_path
        self.window_size = window_size
        self.base_sigma_ai = base_sigma_ai
        self.enable_tflite = enable_tflite

        self.speed_filter: Optional[SpeedFilter] = None
        if self.enable_tflite and os.path.exists(model_path):
            try:
                self.speed_filter = SpeedFilter(model_path=model_path, window_size=window_size)
            except Exception as e:
                print(f"[AICorrectionModule] Warning: Failed to initialize SpeedFilter ({e}). Running in fallback mode.")
                self.speed_filter = None

        self.imu_buffer: List[np.ndarray] = []

    def process_imu_sample(
        self,
        acc_raw: np.ndarray,
        gyro_raw: np.ndarray
    ) -> Tuple[Optional[float], float, float]:
        """
        Process a single incoming 6-DOF IMU sample.

        acc_raw: (3,) Specific force in phone frame (m/s^2)
        gyro_raw: (3,) Angular rate in phone frame (rad/s)

        Returns:
        (ai_speed_estimate_or_None, adaptive_sigma_speed, process_noise_scaling)
        """
        sample = np.hstack([acc_raw, gyro_raw])  # (6,)
        self.imu_buffer.append(sample)
        if len(self.imu_buffer) > self.window_size:
            self.imu_buffer.pop(0)

        if len(self.imu_buffer) < self.window_size:
            return None, self.base_sigma_ai, 1.0

        window = np.array(self.imu_buffer)  # (W, 6)

        # 1. Compute vibration energy / turbulence factor
        acc_vib = np.std(window[:, :3], axis=0)
        total_vib = float(np.linalg.norm(acc_vib))

        # Dynamic process noise scaling: higher vibration => slightly higher process noise Q
        q_scale = 1.0 + min(3.0, total_vib / 1.5)

        # Measurement uncertainty for AI speed: higher vibration => lower trust (higher sigma)
        sigma_speed = self.base_sigma_ai * (1.0 + total_vib / 2.0)

        # 2. Run TFLite inference if available
        pred_speed = None
        if self.speed_filter is not None:
            try:
                pred_speed = self.speed_filter.predict_window(window)
                pred_speed = max(0.0, float(pred_speed))
            except Exception:
                pred_speed = None

        return pred_speed, sigma_speed, q_scale
