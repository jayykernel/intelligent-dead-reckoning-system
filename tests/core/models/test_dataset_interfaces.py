"""
Unit tests for dataset interfaces ensuring leak-free splitting and correct windowing.
"""
import numpy as np
import pytest

from core.models.dataset_interfaces import (
    VehicleClass,
    ImuWindowFeatures,
    TrajectoryGroundTruth,
    TrajectorySample,
    TrajectorySequence,
    DatasetSplitter,
    DatasetAdapter
)


def test_imu_window_features_shape_validation():
    """Test that ImuWindowFeatures raises on incorrect shape."""
    # Correct shape (6, W)
    correct = np.random.rand(6, 100)
    feat = ImuWindowFeatures(data=correct)
    assert feat.data.shape == (6, 100)

    # Wrong shape: (5, W)
    with pytest.raises(ValueError):
        ImuWindowFeatures(data=np.random.rand(5, 100))

    # Wrong shape: (6,)
    with pytest.raises(ValueError):
        ImuWindowFeatures(data=np.random.rand(6))


def test_trajectory_sample_creation():
    """Test creation of a TrajectorySample."""
    feat_data = np.random.rand(6, 50)
    feat = ImuWindowFeatures(data=feat_data)
    truth = TrajectoryGroundTruth(velocity_forward_mps=15.0)
    sample = TrajectorySample(
        features=feat,
        truth=truth,
        timestamp_end_ns=1_000_000_000
    )
    assert sample.features is feat
    assert sample.truth.velocity_forward_mps == 15.0
    assert sample.timestamp_end_ns == 1_000_000_000


def test_trajectory_sequence_duration():
    """Test TrajectorySequence.duration_s property."""
    # Create dummy samples
    feat_data = np.random.rand(6, 10)
    feat = ImuWindowFeatures(data=feat_data)
    truth = TrajectoryGroundTruth(velocity_forward_mps=10.0)
    samples = [
        TrajectorySample(features=feat, truth=truth, timestamp_end_ns=i)
        for i in range(5)
    ]
    seq = TrajectorySequence(
        trajectory_id="trip_001",
        vehicle_class=VehicleClass.CAR,
        samples=samples,
        sample_rate_hz=10.0  # 10 Hz => 0.1s per sample
    )
    # duration_s = len(samples) / sample_rate_hz = 5 / 10 = 0.5s
    assert seq.duration_s == 0.5


def test_dataset_splitter_no_leakage():
    """Test that split_by_trajectory keeps trajectory IDs isolated."""
    # Create three trajectories with distinct IDs
    traj1 = _make_dummy_trajectory("trip_A", VehicleClass.CAR, 5)
    traj2 = _make_dummy_trajectory("trip_B", VehicleClass.MOTORCYCLE, 3)
    traj3 = _make_dummy_trajectory("trip_C", VehicleClass.SCOOTER, 4)
    trajectories = [traj1, traj2, traj3]

    train, val, test = DatasetSplitter.split_by_trajectory(
        trajectories, train_ratio=0.33, val_ratio=0.33, seed=42
    )

    # Collect IDs in each split
    train_ids = {t.trajectory_id for t in train}
    val_ids = {t.trajectory_id for t in val}
    test_ids = {t.trajectory_id for t in test}

    # Ensure no ID appears in more than one split
    assert len(train_ids & val_ids) == 0
    assert len(train_ids & test_ids) == 0
    assert len(val_ids & test_ids) == 0

    # Ensure all IDs are accounted for
    all_ids = train_ids | val_ids | test_ids
    assert all_ids == {"trip_A", "trip_B", "trip_C"}


def test_dataset_adapter_windowing():
    """Test that DatasetAdapter creates correct windows and labels."""
    N = 200  # total samples
    times_s = np.linspace(0, 2, N)  # 2 seconds
    # Simulate constant acceleration and velocity
    accel_v = np.tile([0.1, 0.0, 9.81], (N, 1))  # (N,3)
    gyro_v = np.zeros((N, 3))
    vel_v = np.tile([5.0, 0.0, 0.0], (N, 1))  # forward velocity 5 m/s

    window_size = 20
    stride = 5
    seq = DatasetAdapter.from_continuous_arrays(
        trajectory_id="test_trip",
        vehicle_class=VehicleClass.CAR,
        times_s=times_s,
        accel_v=accel_v,
        gyro_v=gyro_v,
        vel_v=vel_v,
        window_size=window_size,
        stride=stride
    )

    # Number of windows expected: floor((N - window_size) / stride) + 1
    expected_windows = ((N - window_size) // stride) + 1
    assert len(seq.samples) == expected_windows

    # Check first window features shape
    first_feat = seq.samples[0].features.data
    assert first_feat.shape == (6, window_size)
    # First three channels should be accel, next three gyro
    np.testing.assert_allclose(first_feat[:3, :], accel_v[0:window_size].T)
    np.testing.assert_allclose(first_feat[3:, :], gyro_v[0:window_size].T)

    # Check label: forward velocity at end of window
    # For window i, label is vel_v[i+window_size-1, 0]
    for i, sample in enumerate(seq.samples):
        start_idx = i * stride
        end_idx = start_idx + window_size - 1
        expected_v = float(vel_v[end_idx, 0])
        assert sample.truth.velocity_forward_mps == expected_v
        # Check timestamp
        expected_ts = int(times_s[end_idx] * 1e9)
        assert sample.timestamp_end_ns == expected_ts

    # Check sample rate inferred (should be 1/(times_s[1]-times_s[0]) = 1/(2/199) = 99.5)
    dt = times_s[1] - times_s[0]
    expected_rate = 1.0 / dt
    assert seq.sample_rate_hz == pytest.approx(expected_rate, rel=1e-3)


def _make_dummy_trajectory(trip_id: str, vehicle_class: VehicleClass, num_windows: int) -> TrajectorySequence:
    """Helper to create a dummy trajectory with given number of windows."""
    feat_data = np.random.rand(6, 10)
    feat = ImuWindowFeatures(data=feat_data)
    truth = TrajectoryGroundTruth(velocity_forward_mps=5.0)
    samples = [
        TrajectorySample(features=feat, truth=truth, timestamp_end_ns=i)
        for i in range(num_windows)
    ]
    return TrajectorySequence(
        trajectory_id=trip_id,
        vehicle_class=vehicle_class,
        samples=samples,
        sample_rate_hz=10.0
    )