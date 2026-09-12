"""
Dataset interfaces for Machine Learning velocity estimation.
Provides a unified schema for real and synthetic data ingestion,
preventing data leakage and enforcing trajectory-level splitting.
"""
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
from enum import Enum
import numpy as np

class VehicleClass(Enum):
    CAR = "car"
    MOTORCYCLE = "motorcycle"
    SCOOTER = "scooter"
    UNKNOWN = "unknown"

@dataclass
class ImuWindowFeatures:
    """
    Features extracted from an IMU window.
    Shape: (6, window_size) where channels are (ax, ay, az, gx, gy, gz) in Vehicle Frame.
    Units: m/s^2 for accel, rad/s for gyro.
    """
    data: np.ndarray

    def __post_init__(self):
        if self.data.ndim != 2 or self.data.shape[0] != 6:
            raise ValueError(f"Expected shape (6, W), got {self.data.shape}")

@dataclass
class TrajectoryGroundTruth:
    """Ground truth values for a specific trajectory window."""
    velocity_forward_mps: float
    # Optional flags for evaluation
    is_gnss_denied: bool = False

@dataclass
class TrajectorySample:
    """A single matched window and ground truth sample."""
    features: ImuWindowFeatures
    truth: TrajectoryGroundTruth
    timestamp_end_ns: int

@dataclass
class TrajectorySequence:
    """
    A continuous recorded trajectory (trip).
    Data splitting MUST occur at this level, never at the window level,
    to prevent temporal leakage between train and test sets.
    """
    trajectory_id: str
    vehicle_class: VehicleClass
    samples: List[TrajectorySample]
    sample_rate_hz: float

    @property
    def duration_s(self) -> float:
        """Returns the duration of the trajectory in seconds based on sample count / rate."""
        # This is an approximation of duration of valid windows
        return len(self.samples) / self.sample_rate_hz

class DatasetSplitter:
    """Handles statistically sound splitting of trajectories."""

    @staticmethod
    def split_by_trajectory(
        trajectories: List[TrajectorySequence],
        train_ratio: float = 0.7,
        val_ratio: float = 0.15,
        seed: int = 42
    ) -> Tuple[List[TrajectorySequence], List[TrajectorySequence], List[TrajectorySequence]]:
        """
        Splits a list of trajectories into Train/Val/Test strictly by trajectory ID.
        Ensures no windows from the same trip cross into different splits.
        """
        rng = np.random.default_rng(seed)
        shuffled = list(trajectories)
        rng.shuffle(shuffled)

        n = len(shuffled)
        n_train = int(n * train_ratio)
        n_val = int(n * val_ratio)

        train = shuffled[:n_train]
        val = shuffled[n_train:n_train + n_val]
        test = shuffled[n_train + n_val:]
        return train, val, test


class DatasetAdapter:
    """Utility methods to convert raw recordings into uniform TrajectorySequences."""

    @staticmethod
    def from_continuous_arrays(
        trajectory_id: str,
        vehicle_class: VehicleClass,
        times_s: np.ndarray,
        accel_v: np.ndarray,
        gyro_v: np.ndarray,
        vel_v: np.ndarray,
        window_size: int = 100,
        stride: int = 10
    ) -> TrajectorySequence:
        """
        Convert continuous arrays into strongly-typed TrajectorySequence.

        Args:
            accel_v: (N, 3) in m/s^2
            gyro_v: (N, 3) in rad/s
            vel_v: (N, 3) in m/s (we extract [0] for forward velocity)
            window_size: Length of IMU window in samples
            stride: Samples to skip between generated windows
        """
        N = len(times_s)
        samples = []

        for i in range(0, N - window_size + 1, stride):
            w_acc = accel_v[i:i+window_size].T  # (3, W)
            w_gyr = gyro_v[i:i+window_size].T  # (3, W)
            feat_data = np.vstack([w_acc, w_gyr]).astype(np.float32)

            feat = ImuWindowFeatures(data=feat_data)

            # Label is the forward velocity at the END of the window
            v_fwd = float(vel_v[i+window_size-1, 0])
            truth = TrajectoryGroundTruth(velocity_forward_mps=v_fwd)

            t_end_ns = int(times_s[i+window_size-1] * 1e9)

            samples.append(TrajectorySample(
                features=feat,
                truth=truth,
                timestamp_end_ns=t_end_ns
            ))

        # Infer sample rate from consecutive timestamps (approx)
        dt = float(times_s[1] - times_s[0]) if N > 1 else 0.01
        sample_rate = 1.0 / dt if dt > 0 else 100.0

        return TrajectorySequence(
            trajectory_id=trajectory_id,
            vehicle_class=vehicle_class,
            samples=samples,
            sample_rate_hz=sample_rate
        )
