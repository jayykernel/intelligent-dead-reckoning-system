#!/usr/bin/env python3
"""
Diagnostic: Check if ML model is actually working
"""
import sys
import os
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))

from core.models.velocity_estimator import Velocity1DCNN, VelocityEstimatorAPI
from core.models.dataset_generator import SyntheticTrajectoryGenerator

def test_model_loading():
    """Test if model loads and produces reasonable outputs."""
    print("="*80)
    print("ML MODEL DIAGNOSTIC")
    print("="*80)

    # Test 1: Check model file
    model_path = "velocity_model.pth"
    if os.path.exists(model_path):
        file_size = os.path.getsize(model_path)
        print(f"\n[OK] Model file exists: {model_path} ({file_size} bytes)")
    else:
        print(f"\n[FAIL] Model file NOT FOUND: {model_path}")
        return

    # Test 2: Load model directly
    print("\nTest 2: Direct model loading")
    try:
        model = Velocity1DCNN()
        model.load_state_dict(torch.load(model_path, map_location='cpu'))
        model.eval()
        print("[OK] Model loaded successfully")
    except Exception as e:
        print(f"[FAIL] Model loading failed: {e}")
        return

    # Test 3: Generate test trajectory and check predictions
    print("\nTest 3: Model predictions on synthetic data")
    gen = SyntheticTrajectoryGenerator(seed=123)
    traj = gen.generate_straight_accel_decel(duration=10.0, max_speed=20.0)

    accel_v = traj['accel_v']
    gyro_v = traj['gyro_v']
    vel_gt = traj['vel_v'][:, 0]  # forward velocity

    # Build windows
    window_size = 100
    predictions = []
    ground_truths = []

    for i in range(window_size, len(accel_v), 10):  # stride=10
        # Extract window
        window_accel = accel_v[i-window_size:i]
        window_gyro = gyro_v[i-window_size:i]

        # Stack into (1, 6, 100) tensor
        features = np.zeros((1, 6, window_size), dtype=np.float32)
        features[0, 0, :] = window_accel[:, 0]
        features[0, 1, :] = window_accel[:, 1]
        features[0, 2, :] = window_accel[:, 2]
        features[0, 3, :] = window_gyro[:, 0]
        features[0, 4, :] = window_gyro[:, 1]
        features[0, 5, :] = window_gyro[:, 2]

        # Predict
        with torch.no_grad():
            x = torch.from_numpy(features)
            vel_pred, log_var = model(x)
            predictions.append(float(vel_pred.item()))
            ground_truths.append(float(vel_gt[i]))

    predictions = np.array(predictions)
    ground_truths = np.array(ground_truths)
    residuals = predictions - ground_truths

    print(f"Number of predictions: {len(predictions)}")
    print(f"Ground truth range: {ground_truths.min():.2f} to {ground_truths.max():.2f} m/s")
    print(f"Prediction range: {predictions.min():.2f} to {predictions.max():.2f} m/s")
    print(f"Mean ground truth: {ground_truths.mean():.2f} m/s")
    print(f"Mean prediction: {predictions.mean():.2f} m/s")
    print(f"Mean residual: {residuals.mean():.2f} m/s")
    print(f"Residual RMSE: {np.sqrt(np.mean(residuals**2)):.2f} m/s")
    print(f"Max abs residual: {np.abs(residuals).max():.2f} m/s")

    if np.abs(predictions.mean()) < 1.0:
        print("\n[FAIL] FAILURE: Model predictions are near-zero (untrained model)")
    elif np.abs(residuals.mean()) > 5.0:
        print("\n[FAIL] FAILURE: Large systematic bias in predictions")
    elif np.sqrt(np.mean(residuals**2)) > 5.0:
        print("\n[FAIL] FAILURE: RMSE > 5 m/s (model not working)")
    else:
        print("\n[OK] SUCCESS: Model predictions are reasonable")

    # Test 4: Check via VelocityEstimatorAPI
    print("\nTest 4: VelocityEstimatorAPI")
    try:
        api = VelocityEstimatorAPI(model_path=model_path, window_size=100, update_interval=20)
        print(f"[OK] API initialized, is_ready={api.is_ready}")

        # Feed samples
        for i in range(100):
            api.add_vehicle_frame_sample(
                tuple(accel_v[i]),
                tuple(gyro_v[i])
            )

        if api.should_update():
            v_pred, v_var = api.estimate_velocity()
            v_gt = vel_gt[99]
            print(f"  Ground truth at sample 99: {v_gt:.2f} m/s")
            print(f"  API prediction: {v_pred:.2f} m/s")
            print(f"  API variance: {v_var:.2f} (m/s)^2")
            print(f"  Residual: {v_pred - v_gt:.2f} m/s")

            if np.abs(v_pred) < 1.0:
                print("  [FAIL] API producing near-zero predictions")
            else:
                print("  [OK] API producing non-zero predictions")
        else:
            print("  [FAIL] API should_update() returned False")

    except Exception as e:
        print(f"[FAIL] API test failed: {e}")

    print("\n" + "="*80)

if __name__ == "__main__":
    test_model_loading()
