"""
Machine Learning model for forward velocity estimation from inertial windows.
"""

import math
import numpy as np
from typing import Tuple, Dict

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

if TORCH_AVAILABLE:
    class Velocity1DCNN(nn.Module):
        """
        Lightweight 1D Convolutional Neural Network for estimating
        forward velocity and its uncertainty from IMU windows.
        
        Input: (Batch, Channels=6, Seq_Length)
        Channels: ax, ay, az, gx, gy, gz in Vehicle Frame.
        
        Output: (Batch, 2)
        - pred_velocity: Forward speed (m/s)
        - log_var: Log variance of the prediction (for uncertainty)
        """
        def __init__(self, in_channels: int = 6, window_size: int = 100):
            super().__init__()
            self.window_size = window_size
            
            # Simple hierarchical 1D CNN
            self.net = nn.Sequential(
                nn.Conv1d(in_channels, 16, kernel_size=5, padding=2),
                nn.ReLU(),
                nn.MaxPool1d(2), # -> len/2
                
                nn.Conv1d(16, 32, kernel_size=3, padding=1),
                nn.ReLU(),
                nn.MaxPool1d(2), # -> len/4
                
                nn.Conv1d(32, 64, kernel_size=3, padding=1),
                nn.ReLU(),
                nn.AdaptiveAvgPool1d(1) # -> (Batch, 64, 1)
            )
            
            self.fc = nn.Sequential(
                nn.Linear(64, 32),
                nn.ReLU(),
                nn.Linear(32, 2) # vel, log_var
            )
            
        def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
            """
            Args:
                x: (Batch, 6, window_size)
            Returns:
                vel, log_var
            """
            feats = self.net(x)
            feats = feats.view(feats.size(0), -1)
            out = self.fc(feats)
            
            vel = out[:, 0]
            log_var = out[:, 1]
            return vel, log_var

class VelocityEstimatorAPI:
    """
    Deterministic inference API wrapping the ML model.
    Provides fallback mechanisms when data is insufficient.
    """
    def __init__(self, model_path: str = None, window_size: int = 100, update_interval: int = 20):
        self.window_size = window_size
        self.update_interval = update_interval
        self.samples_since_last_update = 0
        self.is_ready = False
        self.buffer = []

        if TORCH_AVAILABLE:
            self.model = Velocity1DCNN(in_channels=6, window_size=window_size)
            if model_path:
                try:
                    self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
                    self.model.eval()
                    self.is_ready = True
                except Exception as e:
                    print(f"Failed to load model: {e}. Running untrained/fallback.")
            else:
                # If no path, we can still run forward pass (with untrained weights or fallback logic)
                self.model.eval()
                self.is_ready = True
        else:
            self.model = None

    def add_vehicle_frame_sample(self, accel_v: Tuple[float, float, float], gyro_v: Tuple[float, float, float]) -> None:
        """Add a sample to the rolling window buffer."""
        self.buffer.append((accel_v, gyro_v))
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
            self.samples_since_last_update += 1

    def should_update(self) -> bool:
        """Check if enough samples have passed since the last update to mitigate correlated residuals."""
        return len(self.buffer) >= self.window_size and self.samples_since_last_update >= self.update_interval

    def estimate_velocity(self) -> Tuple[float, float]:
        """
        Produce a forward velocity estimate and its variance.
        Resets the update counter if an estimate is produced.

        Returns:
            velocity (m/s)
            variance (m/s)^2
        """
        # Fallback 1: Insufficient history
        if len(self.buffer) < self.window_size:
            return 0.0, 100.0 # High uncertainty

        # Fallback 2: Torch not available or model failing
        if not TORCH_AVAILABLE or not self.is_ready:
            # Kinematic fallback: integral of accel roughly?
            # Or just return highly uncertain zero.
            return 0.0, 1000.0

        # We are performing an update, so reset the counter
        self.samples_since_last_update = 0

        # Construct input tensor
        # Shape: (1, 6, window_size)
        inp = np.zeros((1, 6, self.window_size), dtype=np.float32)
        for i in range(self.window_size):
            a, g = self.buffer[i]
            inp[0, 0, i] = a[0]
            inp[0, 1, i] = a[1]
            inp[0, 2, i] = a[2]
            inp[0, 3, i] = g[0]
            inp[0, 4, i] = g[1]
            inp[0, 5, i] = g[2]
            
        with torch.no_grad():
            x = torch.from_numpy(inp)
            vel, log_var = self.model(x)
            
            vel_val = float(vel.item())
            var_val = math.exp(float(log_var.item()))
            
            # Constrain extreme uncertainty blowups
            var_val = np.clip(var_val, 0.01, 100.0)
            
            # Additional heuristic graceful degradation:
            # If standard deviation of gyro is excessive, increase variance penalty
            gyro_y = inp[0, 4, :]
            if np.std(gyro_y) > 2.0:
                var_val += 5.0
                
            return vel_val, float(var_val)
            
    def clear(self):
        """Clear the rolling buffer."""
        self.buffer.clear()
