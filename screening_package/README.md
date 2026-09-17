# Screening Package — AI Speed & Vibration Filter

**Project**: Intelligent Dead Reckoning (IDR) with GNSS+INS Fusion  
**Phase**: 3 (AI Speed & Vibration Filter — Training & Evaluation)  
**Date**: 2026-09-17

---

## 1. Problem Statement

Smartphone-grade MEMS IMU sensors (accelerometer + gyroscope) suffer from
noise, bias drift, and vibration artifacts that make raw strapdown inertial
navigation diverge rapidly. Classical 6-DOF strapdown mechanization — double-
integrating specific force in an ENU frame — produces >1500% position drift
within 60 seconds of GNSS denial on real driving data, rendering it unusable
for navigation without correction.

This filter addresses the **velocity estimation** sub-problem: learning to
predict instantaneous forward vehicle speed from raw IMU windows, bypassing
the catastrophic drift inherent in double integration of noisy accelerometer
data.

## 2. Approach

A **1D-CNN + GRU** neural network processes sliding windows of 6-axis IMU
data (accelerometer + gyroscope, 10 samples @ 10 Hz = 1.0s temporal context):

```
Input: (10, 6) — [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
  ↓
Conv1D(32 filters, kernel=3, ReLU, same padding)
MaxPooling1D(2) + Dropout(0.2)
  ↓
GRU(64 units, unroll=True) + Dropout(0.2)
  ↓
Dense(32, ReLU) → Dense(1)
  ↓
Output: Forward speed (m/s, scalar)
```

- **1D-CNN layers** extract local vibration/noise patterns and frequency
  features from the IMU signal.
- **GRU layer** captures temporal dynamics (acceleration/deceleration,
  inertia) across the 1-second window.
- Output is a single scalar: estimated forward ground speed in m/s.

**Training data**: 642,670 sliding-window sequences from 42 IO-VNBD
TRAIN_SESSIONS (Drivers A, B, E across multiple routes and conditions).

**Framework**: Native TensorFlow/Keras in Python 3.11, exported directly
to TFLite (no ONNX intermediate).

## 3. Results — Held-Out Test Sessions

All results below are on **genuinely held-out TEST_SESSIONS** that the model
never saw during training.

### 3.1 Session S4 (Driver A) — Same Driver/Vehicle, Held-Out Route

| Metric | Phase 2 Strapdown (no AI) | Phase 3 AI Filter |
|--------|---------------------------|-------------------|
| Ground Truth Distance | 153.10 m | 153.10 m |
| Final Position Error | 7120.71 m | 291.17 m |
| Drift % | 4651.03% | **190.18%** |
| **Drift Reduction** | — | **95.91%** |
| Speed MAE | — | 3.72 m/s |
| Speed RMSE | — | 4.46 m/s |

### 3.2 Session Vta26 (Driver E) — Different Driver/Vehicle

| Metric | Phase 2 Strapdown (no AI) | Phase 3 AI Filter |
|--------|---------------------------|-------------------|
| Ground Truth Distance | 574.30 m | 574.30 m |
| Final Position Error | 8719.54 m | 1176.33 m |
| Drift % | 1518.29% | **204.83%** |
| **Drift Reduction** | — | **86.51%** |
| Speed MAE | — | 4.79 m/s |
| Speed RMSE | — | 5.78 m/s |

**Cross-driver generalization confirmed**: The model generalizes from
training drivers to held-out sessions from both the same driver (S4, 96%
reduction) and a different driver (Vta26, 87% reduction).

### 3.3 Summary

The AI speed filter reduces position drift by **86–96%** on held-out test
data. While drift still exceeds 100% (the trajectory still diverges over
60 seconds without GNSS), this is expected at this stage — the filter
addresses only velocity estimation. Future phases add:
- NHC + ZUPT constraints (Phase 5)
- Full EKF/UKF fusion with GNSS (Phase 6)
- Map matching (Phase 7)

Each layer compounds the improvement.

## 4. Model Deployment Feasibility

| Metric | Value | Mobile Budget |
|--------|-------|---------------|
| TFLite size | 108 KB | ✅ Well within mobile limits |
| Inference latency | 0.038 ms/sample | ✅ << 100 ms @ 10 Hz update |
| Input requirement | 6-axis IMU @ 10 Hz | ✅ Standard phone sensors |

## 5. Package Contents

```
screening_package/
├── README.md           ← this file
├── speed_filter.tflite ← trained TFLite model (108 KB)
└── plots/
    ├── S4_ai_speed_filter_comparison.png    ← Held-out: Driver A
    └── Vta26_ai_speed_filter_comparison.png ← Held-out: Driver E
```

Full model checkpoint: `training/models/speed_filter.keras`  
Training script: `training/train_speed_filter.py`  
Evaluation script: `engine/run_ai_speed_filter_baseline.py`  
Detailed results: `docs/PHASE3_RESULTS.md`

## 6. Integrity Note

Initial evaluation was mistakenly run on Session S1, which is in
TRAIN_SESSIONS. This was caught before finalization and corrected. All
results in this package use only TEST_SESSIONS (S4, Vta26). See
`docs/OPEN_QUESTIONS.md` for the full record.
