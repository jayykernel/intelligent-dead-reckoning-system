import os
import sys
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models

sys.path.insert(0, os.path.abspath("."))
from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session
from training.dataset_splits import TRAIN_SESSIONS

def extract_features_from_window(acc_win, gyro_win):
    feats = []

    # Isolate vibration (remove mean gravity / DC rotation)
    acc_vib = acc_win - np.mean(acc_win, axis=0)
    gyro_vib = gyro_win - np.mean(gyro_win, axis=0)

    for arr in [acc_vib, gyro_vib]:
        feats.extend(np.std(arr, axis=0))
        feats.extend(np.ptp(arr, axis=0))
        feats.extend(np.sqrt(np.mean(arr**2, axis=0)))

        # FFT, ignore DC component (index 0)
        fft_vals = np.abs(np.fft.rfft(arr, axis=0))
        if len(fft_vals) > 1:
            feats.extend(np.max(fft_vals[1:], axis=0))
        else:
            feats.extend(np.zeros(3))

    return np.array(feats)

def build_dataset(win_size=20, step_size=10):
    X = []
    y = []
    
    # 1. Car data from IO-VNBD TRAIN_SESSIONS
    print("Loading Car data...")
    for driver, session in TRAIN_SESSIONS[:5]: 
        try:
            s_df, v_df = load_iovnbd_session("data/raw", driver, session)
            synced = preprocess_session(s_df, v_df)
            acc = synced[["acc_x", "acc_y", "acc_z"]].values
            gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
            for i in range(0, len(acc) - win_size, step_size):
                X.append(extract_features_from_window(acc[i:i+win_size], gyro[i:i+win_size]))
                y.append(0)
        except: continue
            
    # 2. Two-Wheeler data
    print("Loading Two-Wheeler data...")
    for session in ["session3", "session4"]:
        synced = load_two_wheeler_session("data/raw/two_wheeler", session)
        acc = synced[["acc_x", "acc_y", "acc_z"]].values
        gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
        for i in range(0, len(acc) - win_size, step_size):
            X.append(extract_features_from_window(acc[i:i+win_size], gyro[i:i+win_size]))
            y.append(1)
    
    return np.array(X), np.array(y)

def main():
    X, y = build_dataset()
    
    # Simple MLP
    model = models.Sequential([
        layers.Input(shape=(X.shape[1],)),
        layers.Dense(32, activation='relu'),
        layers.Dropout(0.2),
        layers.Dense(1, activation='sigmoid')
    ])
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
    
    indices = np.arange(len(X))
    np.random.seed(42)
    np.random.shuffle(indices)
    X = X[indices]
    y = y[indices]
    model.fit(X, y, epochs=10, batch_size=32, validation_split=0.2)

    
    # Export
    os.makedirs("training/models", exist_ok=True)
    model.save("training/models/vehicle_classifier.keras")
    
    # TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    with open("training/models/vehicle_classifier.tflite", "wb") as f:
        f.write(tflite_model)
    print("Saved vehicle_classifier to .keras and .tflite")

if __name__ == "__main__":
    main()
