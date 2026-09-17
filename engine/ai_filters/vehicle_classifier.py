"""
Vehicle-Type Classifier (N6)

Uses vibration/spectral characteristics over a sliding window
to classify vehicle type into Car vs Two-Wheeler.
Implements hysteresis / sustained window agreement to prevent state thrashing.
"""

import numpy as np
import os
from typing import Optional

try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        Interpreter = None

class VehicleClassifier:
    def __init__(self, model_path: str = "training/models/vehicle_classifier.tflite", window_size: int = 20, min_sustained_agreements: int = 5):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        if Interpreter is None:
            raise ImportError("Neither ai_edge_litert nor tensorflow.lite is installed.")

        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.window_size = window_size
        self.min_sustained_agreements = min_sustained_agreements

        self.recent_predictions = []
        self.current_class = "car" # Default

    def extract_features(self, acc_win: np.ndarray, gyro_win: np.ndarray) -> np.ndarray:
        feats = []
        acc_vib = acc_win - np.mean(acc_win, axis=0)
        gyro_vib = gyro_win - np.mean(gyro_win, axis=0)

        for arr in [acc_vib, gyro_vib]:
            feats.extend(np.std(arr, axis=0))
            feats.extend(np.ptp(arr, axis=0))
            feats.extend(np.sqrt(np.mean(arr**2, axis=0)))
            fft_vals = np.abs(np.fft.rfft(arr, axis=0))
            if len(fft_vals) > 1:
                feats.extend(np.max(fft_vals[1:], axis=0))
            else:
                feats.extend(np.zeros(3))
        return np.array(feats, dtype=np.float32)

    def predict_window(self, acc_win: np.ndarray, gyro_win: np.ndarray) -> str:
        """
        Runs inference on a single (W, 3) window and updates sustained state.
        """
        feat = self.extract_features(acc_win, gyro_win).reshape(1, -1)
        self.interpreter.set_tensor(self.input_details[0]['index'], feat)
        self.interpreter.invoke()
        prob = self.interpreter.get_tensor(self.output_details[0]['index'])[0][0]

        raw_pred = "two_wheeler" if prob > 0.5 else "car"
        self.recent_predictions.append(raw_pred)
        if len(self.recent_predictions) > self.min_sustained_agreements:
            self.recent_predictions.pop(0)

        # Hysteresis: Require sustained agreement to switch classes
        if len(self.recent_predictions) == self.min_sustained_agreements:
            if all(p == raw_pred for p in self.recent_predictions):
                self.current_class = raw_pred

        return self.current_class
