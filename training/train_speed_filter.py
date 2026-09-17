"""
train_speed_filter.py

Phase 3 - Training the AI Speed & Vibration Filter.

Implements a combined 1D-CNN + GRU architecture to predict forward velocity
from raw IMU windows.

Architecture:
- Input window (size N=10 samples): (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
- 1D-CNN layers for feature extraction (vibration patterns, noise reduction)
- GRU layers for sequence modeling (velocity, inertia)
- Output: Single scalar (forward speed, m/s)

Trained on preprocessed IO-VNBD sessions.
"""

import os
import argparse
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models

# Import dataset splits
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dataset_splits import DATASET_SPLITS
from data_loader import load_iovnbd_session, preprocess_session

def create_model(window_size: int = 10, n_features: int = 6):
    """Simple CNN-GRU architecture for speed estimation."""
    model = models.Sequential([
        layers.Input(shape=(window_size, n_features)),
        # Feature extraction
        layers.Conv1D(filters=32, kernel_size=3, activation='relu', padding='same'),
        layers.MaxPooling1D(pool_size=2),
        layers.Dropout(0.2),
        # Sequence modeling
        layers.GRU(64, return_sequences=False, unroll=True),
        layers.Dropout(0.2),
        # Regression head
        layers.Dense(32, activation='relu'),
        layers.Dense(1)  # Predicted speed (m/s)
    ])
    model.compile(optimizer='adam', loss='mse', metrics=['mae'])
    return model

def prepare_sequences(data: np.ndarray, gt_speed: np.ndarray, window_size: int = 10):
    """
    Create sliding window sequences for CNN/GRU input.
    data: (N, 6) [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
    gt_speed: (N,)
    """
    if len(data) <= window_size:
        return np.array([]), np.array([])

    # We can optimize this with stride_tricks, but a loop is fine for now
    X = np.lib.stride_tricks.sliding_window_view(data, (window_size, data.shape[1])).squeeze(axis=1)
    # Target is the speed at the *end* of the window
    y = gt_speed[window_size - 1:]

    # Trim X to match y length (sliding_window_view creates length N - window_size + 1)
    # y is shape (N - window_size + 1,)
    return X, y

def run(data_root: str, model_save_path: str):
    print("[1/3] Loading training data...")
    X_train_list, y_train_list = [], []

    # Using the TRAIN_SESSIONS split from dataset_splits.py
    for driver, session in DATASET_SPLITS['train']:
        try:
            s_df, v_df = load_iovnbd_session(data_root, driver, session)
            synced = preprocess_session(s_df, v_df, target_dt=0.1)

            # Features
            features = synced[["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]].values
            speed = synced["gt_speed"].values

            X_s, y_s = prepare_sequences(features, speed)
            if len(X_s) > 0:
                X_train_list.append(X_s)
                y_train_list.append(y_s)
        except Exception as e:
            print(f"      Warning: Could not process {driver}/{session}: {e}")

    if not X_train_list:
        raise ValueError("No training data found. Ensure raw data is in " + data_root)

    X_train = np.concatenate(X_train_list)
    y_train = np.concatenate(y_train_list)

    print(f"      Training data shape: X={X_train.shape}, y={y_train.shape}")

    print("[2/3] Training model...")
    model = create_model()
    # Simple training loop. In production, we'd use checkpoints, early stopping, etc.
    early_stop = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=3, restore_best_weights=True)
    model.fit(X_train, y_train, epochs=10, batch_size=128, validation_split=0.1, callbacks=[early_stop], verbose=2)

    print("[3/3] Saving model and exporting to TFLite...")
    os.makedirs(os.path.dirname(model_save_path), exist_ok=True)
    # Keras v3 saves in .keras format
    keras_path = model_save_path + ".keras"
    model.save(keras_path)

    # Export TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    tflite_path = model_save_path + ".tflite"
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    # Check size
    tflite_size_kb = os.path.getsize(tflite_path) / 1024
    print(f"      Model saved to: {keras_path}")
    print(f"      TFLite exported: {tflite_path} ({tflite_size_kb:.1f} KB)")

def main():
    parser = argparse.ArgumentParser(description="Phase 3: Train speed filter")
    parser.add_argument("--data-root", default="data/raw", help="Path to raw data")
    parser.add_argument("--model-save", default="training/models/speed_filter", help="Base path to save model")
    args = parser.parse_args()

    # Ensure current directory matches repo root or adjust paths if run from /training
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    data_root = os.path.join(repo_root, args.data_root) if not os.path.isabs(args.data_root) else args.data_root
    model_save = os.path.join(repo_root, args.model_save) if not os.path.isabs(args.model_save) else args.model_save

    run(data_root, model_save)

if __name__ == "__main__":
    main()
