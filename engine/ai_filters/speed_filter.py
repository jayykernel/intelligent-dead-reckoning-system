"""
engine/ai_filters/speed_filter.py

Phase 3 - AI Speed & Vibration Filter inference engine.

Loads the exported TFLite speed filter model and runs inference
on sliding windows of IMU samples (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
to produce estimated forward vehicle speed (m/s).
"""

import os
import numpy as np
from typing import Optional

try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        Interpreter = None


class SpeedFilter:
    """TFLite-based AI Speed and Vibration Filter."""

    def __init__(self, model_path: str, window_size: int = 10):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")

        if Interpreter is None:
            raise ImportError("Neither ai_edge_litert nor tensorflow.lite is installed.")

        self.window_size = window_size
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_index = self.input_details[0]['index']
        self.output_index = self.output_details[0]['index']

        # Ring buffer for online streaming inference
        self.buffer = []

    def predict_window(self, window: np.ndarray) -> float:
        """
        Run inference on a single window of shape (window_size, 6) or (1, window_size, 6).
        Returns predicted scalar forward speed (m/s).
        """
        if window.ndim == 2:
            input_data = np.expand_dims(window, axis=0).astype(np.float32)
        else:
            input_data = window.astype(np.float32)

        self.interpreter.set_tensor(self.input_index, input_data)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_index)
        return float(output[0, 0])

    def predict_sequence(self, imu_data: np.ndarray) -> np.ndarray:
        """
        Run inference over a full trajectory of shape (N, 6).
        For samples before window_size, pads or repeats early predictions.
        Returns predicted speed array of shape (N,).
        """
        N = len(imu_data)
        speeds = np.zeros(N, dtype=np.float32)

        if N < self.window_size:
            return speeds

        # Sliding window predictions
        windows = np.lib.stride_tricks.sliding_window_view(
            imu_data, (self.window_size, imu_data.shape[1])
        ).squeeze(axis=1)

        for i, win in enumerate(windows):
            idx = i + self.window_size - 1
            pred_speed = self.predict_window(win)
            # Ensure non-negative speed
            speeds[idx] = max(0.0, pred_speed)

        # Backfill initial window steps with first valid prediction
        speeds[:self.window_size - 1] = speeds[self.window_size - 1]

        return speeds
