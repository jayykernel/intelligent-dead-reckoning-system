"""
Script to train the Forward Velocity ML model on synthetic data.
Uses trajectory-level splitting to prevent data leakage.
"""

import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import numpy as np

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.models.dataset_generator import SyntheticTrajectoryGenerator
from core.models.velocity_estimator import Velocity1DCNN
from core.models.dataset_interfaces import (
    DatasetAdapter,
    DatasetSplitter,
    TrajectorySequence,
    VehicleClass
)


def gaussian_nll_loss(pred, log_var, target):
    """Negative log likelihood for Gaussian uncertainty prediction."""
    var = torch.exp(log_var)
    loss = 0.5 * ((pred - target)**2 / var + log_var)
    return loss.mean()


def prepare_data_from_sequences(
    sequences: list[TrajectorySequence]
) -> tuple[np.ndarray, np.ndarray]:
    """
    Extract features and targets from a list of TrajectorySequence objects.
    Returns:
        features: np.ndarray of shape (N, 6, W)
        targets: np.ndarray of shape (N,)
    """
    feats = []
    tgts = []
    for seq in sequences:
        for sample in seq.samples:
            feats.append(sample.features.data)
            tgts.append(sample.truth.velocity_forward_mps)
    return np.stack(feats), np.array(tgts)


def train(epochs: int = 10, save_path: str = "velocity_model.pth"):
    torch.manual_seed(42)
    np.random.seed(42)

    print("Generating synthetic trajectories...")
    gen = SyntheticTrajectoryGenerator(seed=42)
    # Generate trajectories for splitting
    raw_trajectories = gen.generate_random_dataset(num_trajectories=60, length_sec=10.0)

    # Convert raw dicts to TrajectorySequence objects
    sequences = []
    for i, traj in enumerate(raw_trajectories):
        seq = DatasetAdapter.from_continuous_arrays(
            trajectory_id=f"traj_{i:03d}",
            vehicle_class=VehicleClass.CAR,  # synthetic data assumes car
            times_s=traj['time'],
            accel_v=traj['accel_v'],
            gyro_v=traj['gyro_v'],
            vel_v=traj['vel_v'],
            window_size=100,
            stride=10
        )
        sequences.append(seq)

    # Split at trajectory level (no leakage)
    train_seqs, val_seqs, _ = DatasetSplitter.split_by_trajectory(
        sequences, train_ratio=0.7, val_ratio=0.15, seed=42
    )

    print(f"Number of trajectories: Train={len(train_seqs)}, Val={len(val_seqs)}, Test={len(_)}")

    # Prepare window-level datasets
    X_train, y_train = prepare_data_from_sequences(train_seqs)
    X_val, y_val = prepare_data_from_sequences(val_seqs)

    print(f"Training windows: {X_train.shape[0]}, Validation windows: {X_val.shape[0]}")

    # Convert to PyTorch tensors
    train_dataset = TensorDataset(
        torch.from_numpy(X_train).float(),
        torch.from_numpy(y_train).float()
    )
    val_dataset = TensorDataset(
        torch.from_numpy(X_val).float(),
        torch.from_numpy(y_val).float()
    )

    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)

    model = Velocity1DCNN()
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    print("Training...")
    for epoch in range(epochs):
        model.train()
        train_loss = 0.0
        for x, y in train_loader:
            optimizer.zero_grad()
            vel, log_var = model(x)
            loss = gaussian_nll_loss(vel, log_var, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        # Validation
        model.eval()
        val_loss = 0.0
        val_rmse = 0.0
        with torch.no_grad():
            for x, y in val_loader:
                vel, log_var = model(x)
                loss = gaussian_nll_loss(vel, log_var, y)
                val_loss += loss.item()
                val_rmse += torch.mean((vel - y)**2).item()

        train_loss /= len(train_loader)
        val_loss /= len(val_loader)
        val_rmse = np.sqrt(val_rmse / len(val_loader))

        print(f"Epoch {epoch+1}/{epochs} | Train NLL: {train_loss:.4f} | Val NLL: {val_loss:.4f} | Val RMSE: {val_rmse:.4f} m/s")

    torch.save(model.state_dict(), save_path)
    print(f"Model saved to {save_path}")


if __name__ == "__main__":
    train()