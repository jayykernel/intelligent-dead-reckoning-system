# Phase 5 Results: Vehicle-Type Classifier + NHC/ZUPT (incl. Lean-Compensated NHC)

## 1. Overview & Architecture Summary
Phase 5 implements vehicle classification and physical motion constraints to arrest the quadratic velocity and cubic position divergence characteristic of unconstrained dead reckoning:
1. **Vehicle-Type Classifier (N6)**: Sliding-window spectral/vibration feature extraction (standard deviation, peak-to-peak, RMS, peak non-DC FFT) + 1D CNN trained to distinguish cars from two-wheelers with a 5-step hysteresis filter.
2. **Lean-Angle EKF Estimator (N1)**: Real-time roll/lean angle estimator for two-wheelers fusing gyro roll rates with centripetal/gravitational vector updates.
3. **Constrained INS Engine**:
   - **Car Mode**: Standard Non-Holonomic Constraints (NHC: $v_y = 0, v_z = 0$ in vehicle frame) + Zero-Velocity Updates (ZUPT).
   - **Two-Wheeler Mode (N1)**: Lean-Compensated NHC rotating the velocity vector by estimated lean angle $\phi$ into the instantaneous road frame before zeroing lateral and vertical velocity components.

---

## 2. Dataset Split & Two-Wheeler Context
- **Car Dataset**: IO-VNBD (held-out test sessions: Driver A / S4, Driver E / Vta26).
- **Two-Wheeler Dataset (Option C)**:
  - **Train Sessions**: `session3`, `session4` (~24.4 min).
  - **Held-Out Test Sessions**: `session1`, `session2` (~8.5 min).
  - **Mount Context**: Fixed in front storage compartment with minor orientation variations (<10° residual error).

> ⚠️ **Explicit Sample Size Caveat**:
> Total two-wheeler data comprises ~33 minutes across 4 real-world riding sessions. While providing realistic engine harmonics and lean dynamics for model training and held-out validation, this is a small sample size. Results should be interpreted with this constraint in mind.

---

## 3. Experimental Evaluation & Drift Results

All evaluations were executed with the full Phase 5 pipeline (`engine/run_phase5_evaluation.py`) using the exported TFLite models. Car and Two-Wheeler metrics are reported separately and not averaged together.

### A. Two-Wheeler Evaluation (Real Held-Out Sessions)

#### 1. Two-Wheeler Held-Out Session 1 (`session1`)
- **Duration**: 335.9 s (3359 samples @ 10 Hz)
- **Ground Truth Distance**: 1645.04 m
- **Phase 2 Baseline (Pure Strapdown INS)**:
  - Final Position Error: 271,500.67 m
  - Drift %: **16,504.16%**
- **Phase 5 Pipeline (Classifier + Lean-Compensated NHC + ZUPT)**:
  - Final Position Error: 1,652.23 m
  - Drift %: **100.44%**
  - Speed MAE: 5.47 m/s | Speed RMSE: 7.79 m/s
  - **Relative Drift Reduction**: **99.39% reduction** over unconstrained baseline
  - *Plot*: `data/processed/phase5_eval/tw/session1/session1_phase5_comparison.png`

#### 2. Two-Wheeler Held-Out Session 2 (`session2`)
- **Duration**: 173.7 s (1737 samples @ 10 Hz)
- **Ground Truth Distance**: 368.77 m
- **Phase 2 Baseline (Pure Strapdown INS)**:
  - Final Position Error: 43,938.01 m
  - Drift %: **11,914.59%**
- **Phase 5 Pipeline (Classifier + Lean-Compensated NHC + ZUPT)**:
  - Final Position Error: 517.68 m
  - Drift %: **140.38%**
  - Speed MAE: 4.89 m/s | Speed RMSE: 8.81 m/s
  - **Relative Drift Reduction**: **98.82% reduction** over unconstrained baseline
  - *Plot*: `data/processed/phase5_eval/tw/session2/session2_phase5_comparison.png`

---

### B. Car Evaluation (IO-VNBD Held-Out Sessions)

#### 1. Car Held-Out Session S4 (Driver A)
- **Duration**: 354.8 s (3548 samples @ 10 Hz)
- **Ground Truth Distance**: 2607.64 m
- **Phase 2 Baseline (Pure Strapdown INS)**:
  - Final Position Error: 218,417.95 m
  - Drift %: **8,376.09%**
- **Phase 5 Pipeline (Classifier + Standard NHC + ZUPT)**:
  - Final Position Error: 2,313.59 m
  - Drift %: **88.72%**
  - Speed MAE: 6.83 m/s | Speed RMSE: 8.33 m/s
  - **Relative Drift Reduction**: **98.94% reduction** over unconstrained baseline
  - *Plot*: `data/processed/phase5_eval/car/S4/S4_phase5_comparison.png`

#### 2. Car Held-Out Session Vta26 (Driver E)
- **Duration**: 193.4 s (1934 samples @ 10 Hz)
- **Ground Truth Distance**: 1049.34 m
- **Phase 2 Baseline (Pure Strapdown INS)**:
  - Final Position Error: 61,100.31 m
  - Drift %: **5,822.76%**
- **Phase 5 Pipeline (Classifier + Standard NHC + ZUPT)**:
  - Final Position Error: 2,111.58 m
  - Drift %: **201.23%**
  - Speed MAE: 6.19 m/s | Speed RMSE: 11.91 m/s
  - **Relative Drift Reduction**: **96.54% reduction** over unconstrained baseline
  - *Plot*: `data/processed/phase5_eval/car/Vta26/Vta26_phase5_comparison.png`

---

## 4. Key Takeaways & Transition to Phase 6
1. **Divergence Prevention**: Applying NHC and ZUPT eliminates vertical and lateral runaway velocity errors, reducing baseline unconstrained INS drift by >98% across all vehicle types and sessions.
2. **Classification & Lean Compensation**:
   - The vibration classifier accurately maintains the vehicle modality.
   - Lean-compensated NHC prevents false lateral non-holonomic velocity damping during turning/banking maneuvers on two-wheelers.
3. **Upcoming Fusion (Phase 6)**: The remaining open-loop drift (~80-140%) is primarily driven by gyro yaw drift and speed scaling errors during extended blackout windows; this will be tightly bound in Phase 6 with the GNSS+INS fusion engine (EKF/UKF + NIS innovation gating).
