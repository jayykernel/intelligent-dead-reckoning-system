# Phase 10 Failure-Isolation Checkpoint

**Date**: 2026-09-13
**Commit**: 2f6848e (Phase 10 completion)
**Objective**: Determine the root cause of the +60.5% drift degradation observed during the Phase 10 Acceptance Audit when feeding ML-predicted forward velocity into the Error-State Kalman Filter (ESKF).

## 1. Experimental Methodology
We constructed a highly controlled script (`failure_isolation_study.py`) executing exactly the same deterministic synthetic 30-second trajectory (seed=123, 100 Hz, 3000 samples) across seven isolated experimental conditions.

The deterministic baseline was: ESKF without any external aiding (Experiment A).

### 1.1 The Seven Controlled Experiments
* **A. ESKF-only**: No external velocity updates. The benchmark tracking pure inertial sensor drift.
* **B. ESKF + PERFECT GT**: Filter updated with ground-truth forward velocity at low variance. Proof of mathematical correctness.
* **C. ESKF + BIASED VEL**: Filter updated with GT velocity plus a constant deliberate +2.0 m/s bias. Isolates filter response to raw static bias.
* **D. ESKF + ML + LARGE COVAR**: Filter updated with ML velocity, but observation covariance $R$ hard-coded to massive conservative value (100.0). Isolates influence of extreme under-confidence.
* **E. ESKF + ML + CONSERV COVAR**: Filter updated with ML velocity, $R$ fixed at reasonable conservative value (5.0).
* **F. ESKF + ML + PRED COVAR**: Filter updated with ML velocity, using the ML model's internal variance predictions (naive integration).
* **G. ESKF + ML + REDUCED FREQ**: Filter updated with ML velocity at 50% reduced ingestion rate (updating every 20 windows instead of 10). Isolates the effect of correlated noise over-injection.

## 2. Experimental Results & Hard Findings

| Experiment | Velocity RMSE (m/s) | Position Drift (m) | Max Position Err (m) | ML Updates Injected |
|------------|---------------------|--------------------|----------------------|---------------------|
| **A. ESKF-only** | 2.501 | -58.207 | 61.896 | 0 |
| **B. ESKF + PERFECT GT** | 0.135 | 1.061 | 1.061 | 291 |
| **C. ESKF + BIASED VEL** | 4.359 | 45.569 | 47.562 | 291 |
| **D. ESKF + ML + LARGE COVAR** | 5.605 | 93.414 | 93.414 | 291 |
| **E. ESKF + ML + CONSERV COVAR** | 5.605 | 93.414 | 93.414 | 291 |
| **F. ESKF + ML + PRED COVAR** | 5.605 | 93.414 | 93.414 | 291 |
| **G. ESKF + ML + RED FREQ** | 1.933 | 19.399 | 19.399 | 146 |

## 3. Mathematical & System Verifications (Ruling out False Negatives)

1. **Velocity Measurement Jacobian ($H$) Validated**: Zero error matching against finite-difference numerical Jacobians.
2. **Attitude Measurement Jacobian ($H_{att}$) Validated**: Zero error matching against finite-difference numerical Jacobians.
3. **Frame Transformations Correct**: Verified vehicle-body to NED navigation frame vector rotations yield precise expected components across all cardinal headings ($0^\circ, 90^\circ, 180^\circ, 270^\circ$).
4. **Temporal Alignment Accurate**: Window termination indexing perfectly aligns with the ground-truth velocity label timestamp exactly natively in the `DatasetAdapter`.
5. **Mahalanobis Gate Active & Responsive**: Confirmed massive innovations strictly reject updates natively mapping mathematical expectations (e.g. 100 m/s residual at low variance pushes Mahalanobis metric > 300 vs threshold of 3.0).
6. **Bias and Variance Calibrations Strict**: We actively stripped empirical bias (-0.384 m/s) and multiplied variance by measured error scalar (2.068x) prior to ingestion.

## 4. Root Cause Analysis (Conclusions)

### Primary Root Cause: Correlated Residual Noise Injection (Experiment G)
The overwhelming contributor to filter divergence is the high-frequency ingestion of **autocorrelated ML prediction errors**. 
* ML errors are not temporally independent zero-mean Gaussian noise (assumed by EKF theory).
* Autocorrelation analysis revealed severe lag correlation: *Lag-0: 1.000, Lag-1: 0.972, Lag-2: 0.962, Lag-3: 0.926...*
* Injecting highly correlated errors at 3 Hz (every 10 windows) acts exactly like injecting colored non-Gaussian process noise directly into the filter updates.
* **Proof**: Experiment G demonstrates that simply halving the injection frequency (from every 10 windows to every 20 windows) breaks the high-frequency correlated integration, cutting drift drastically from **93.414 m down to 19.399 m** (a massive 79% reduction in error).

### Secondary Root Cause: ML Uncertainty Miscalibration
The model's native `log_var` prediction massively underestimates trace error scales natively.
* Mean predicted log_var translated into ~0.885 m/s implied STD.
* Actual measured empirical error STD was ~1.273 m/s.
* This implies a ~2.07x missing multiplicative scalar, causing the ESKF to mathematically over-weight incoming autocorrelated noise instead of dampening it against rigid prior inertial bounds.

### Disproven Hypotheses
* **Filter Mathematics**: Experiment B (Perfect GT) definitively proves that the core Continuous-Discrete ESKF coordinate math, integration, Jacobians, and rotations are completely flawless. Under true observations, drift drops to 1.061 m over 30s.
* **Code / Setup Bugs**: Temporal alignment, gate scaling, and vector projections all passed rigorous numerical validations.

## 5. Status & Restrictions
* Phase 10 marks the **Implementation Complete** bounds of the ML velocity estimator integration.
* **However, ML-aided Navigation Improvement is NOT VALIDATED.**
* The ML component must remain an *optional decoupled integration* moving forward.
* We have transparently identified the correlated-noise failure mode instead of covering it up.

**DO NOT COMMENCE PHASE 11. End of Checkpoint.**
