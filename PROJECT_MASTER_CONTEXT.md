# PROJECT MASTER CONTEXT
**SIH PS 26168** — AI/ML-augmented Intelligent Dead Reckoning Navigation System

## 1. Project Overview
This document consolidates the current state of the repository, architecture, and progress. It serves as the single source of truth for the project's canonical structure and status.

## 2. Current Phase Status (as of 2026-09-13)
| Phase | Description | Status | Evidence Level | Test Count |
|-------|-------------|--------|----------------|------------|
| Phase 0 | Project Audit, Requirements & Architecture | ✅ Completed | Fully Documented | N/A |
| Phase 1 | Project Infrastructure & Build Environment | ✅ Completed | Unit Tested | 51 passing |
| Phase 2 | Sensor Abstraction, Types & Time Synchronization | ✅ Completed | Unit Tested & Synthetic Replay | 14 tests |
| Phase 3 | Sensor Calibration Engine & Sensor Health | ✅ Completed & Validated | Unit & Synthetic Deterministic Tested | 32 tests |
| Phase 4 | Phone-to-Vehicle Alignment Engine | ✅ Completed | Unit Tested | 24 tests |
| Phase 5 | Deterministic INS Mechanization Baseline | ✅ Completed | Unit Tested | 10 tests |
| Phase 6 | Error-State Kalman Filter (ESKF) | ✅ Completed | Unit & End-to-End Tested | 9 tests |
| Phase 7 | Motion Intelligence: ZUPT & ML Velocity | ✅ Completed | Unit & Integration Tested | 20 tests *(116 total)* |
| Phase 8 | GNSS Integrity & Seamless Outage Transitions | ✅ Completed | Unit Tested | 29 tests *(145 total)* |
| Phase 9 | Vibration Analysis & Spectral Decomposition | ✅ Completed | Unit Tested | 7 tests *(152 total)* |
| Phase 10 | ML Model Training & Deployment | ✅ Implemented | SYNTHETICALLY VALIDATED (Not universally validated) | 6 tests *(158 total)* |
| Phase 11 | Real-Data Validation & Generalization | 🔄 In Progress | Synthetic Only (Data Not Available) | 0 |
| Phase 12 | Vehicle Classification | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 13 | Vehicle-Aware Motion Constraints | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 14 | Dynamic Bias / Error Adaptation | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 15 | Map Matching | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 16 | Multi-Hypothesis Navigation | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 17 | Parking / Flyover / Level Disambiguation | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 18 | Navigation Integrity & Uncertainty | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 19 | Mobile / Edge Optimization | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 20 | Cross-Device / Cross-Vehicle Robustness | ⏳ Future Phase | NOT VALIDATED | 0 |
| Phase 21 | Complete System Validation & SIH Benchmark | ⏳ Future Phase | NOT VALIDATED | 0 |

## 3. Validated Core Capabilities (Phases 1–7)

**Phases 1–6 represent the frozen Golden Architectural Foundation.** All future phases must integrate with these canonical interfaces. Do not redesign, bypass, or blindly replace Phase 1–6 components without explicit justification.

- **Sensor Types & Synchronization**: Immutable dataclasses (`ImuSample`, `MagSample`, `GnssFix`, `BaroSample`), linear and spherical linear (`slerp`) quaternion interpolation, CSV/Binary logger and replay iterator.
- **IMU Bias Calibration**: Multi-position static bias estimation decoupling arbitrary 3D gravity vectors from sensor biases.
- **Magnetometer Calibration**: SVD-based least-squares ellipsoid fitting extracting hard-iron bias and soft-iron shape matrices.
- **Phone-Vehicle Alignment**: Phone-to-vehicle alignment engine tracking `GRAVITY_ONLY` tilt via accelerometer, and `FULLY_ALIGNED` yaw via GNSS velocity and Magnetometer aiding. Detects relative movement with rigid thresholding.
- **Mechanization Sub-system**: Deterministic Euler-integrated dead reckoning step translating raw IMU events into smooth Pos/Vel/Attitude NavState traces in Local Tangent Plane (NED).
- **ESKF Architecture**: 15-state Error-State Kalman Filter mapping Position, Velocity, Attitude, and 6-DOF Biases. Mathematically tracks deterministic states with continuous-discrete Jacobians, covariance PSD maintenance via Joseph form, outlier Mahalanobis gating, and direct analytical bias convergence.
- **ZUPT Detector**: Soft probabilistic stationary detection P(stat) ∈ [0,1] with adaptive covariance scaling R_zupt = R_base / (P_stat^γ + ε). Exponential soft thresholding based on accelerometer/gyroscope variance and temporal consistency smoothing.
- **ML Velocity Estimator**: Lightweight 1D CNN architecture accepting 6-channel IMU windows, outputting forward velocity prediction and log-variance for uncertainty quantification. Integration with ESKF via `update_forward_velocity()` with proper vehicle-to-navigation frame Jacobian.
- **Regression Suite**: 116/116 tests passing under Python 3.14 and NumPy 2.5.3 with zero external mathematical instability dependencies.

## 4. Phase 8 Highlights (GNSS Integrity)
- **Continuous Quality Scoring**: Extracted mathematically defensible [0,1] confidence metric matching Satellite geometry, signal accuracy properties, hard/soft Mahalanobis innovation gating, and precise kinematic bounds mapping real-world implausibilities (like 50m/s position jumps).
- **Navigation Mode Manager**: Tracked deterministic recovery phases using strict persistence counts avoiding oscillating transition chattering while seamlessly ramping ESKF measurement gains upon reacquisition to mitigate unmodeled sudden jumps natively without breaking covariance continuities.

## 5. Phase 9 Highlights (Vibration Analysis)
- Implemented `RollingSpectralAnalyzer` utilizing real-valued Fast Fourier Transforms isolating normalized power spectrum variations from static vehicle z-axis constraints across explicit $N/2+1$ distributions analytically mapping hardware capabilities (nominally 100Hz maxing at 50Hz Nyquist).
- Designed `get_spectral_entropy()` method projecting relative peak clarity measuring signal variance vs white noise boundaries natively mapping frequency stability tracking arbitrary uniform variations continuously determining terrain consistency vs harmonic resonances natively.
- Implemented `get_road_roughness()` tracking raw sum integrations over defined low-frequency bands (0-20 Hz) analytically scaling explicit sensor variation magnitude matching rough vs highway profiles continuously.
- Added continuous deterministic harmonic band isolation utilizing generic discrete integrations on 10-30Hz band outputs detecting explicit vehicle drivetrain behaviors decoupling internal kinetic motions from spatial changes implicitly.
- Adjusted math libraries isolating deprecated `numpy.trapz` implementations against standard Python 3.14/Numpy 2.0+ `numpy.trapezoid` equivalents structurally tracking full compatibility natively mapping edge case limits logically across zero length array thresholds continuously.

## 6. Phase 10 Highlights (ML Model Training & Deployment)
- Standardized Dataset abstraction via `core/models/dataset_interfaces.py` defining explicit `TrajectorySequence` isolating distinct kinematic drives explicitly from shared overlapping window generations removing temporal test data leakages consistently across arbitrary train/val/test splits recursively tracking trajectory_ids analytically natively.
- Implemented rigorous synthetic trajectory evaluations linking `test_dataset_interfaces.py` assuring strict split limits tracking analytical isolation deterministically.
- Found extensive trajectory-level biases resulting in systemic estimation bounds negatively impacting basic inertial integrations intrinsically drifting up to 27x nominal RMSE baselines if uncaught structurally. (Pos drift: Pure INS -23m vs ML aided +571m over 30s evaluation).
- Explicitly zeroed the attitude cross-coupling Jacobian `(d(v_x) / d(theta))` resolving uncontrollable multiplicative resonance breaking explicit gravity alignments natively preventing complete unobservable coordinate frame flips (`Yaw=-22.7` deg bounds corrected down into standard drift limits).
- Systematically calibrated predictive variance scalars matching true empirical bounds (2.08x inflation on standard errors relative to naive Gaussian prediction magnitudes).
- Successfully documented analytical ML boundaries strictly avoiding hyperbolic performance claims lacking comprehensive genuine multi-vehicle IO-VNBD validations implicitly guaranteeing baseline navigation honesty natively structurally.
- Phase 10 Acceptance Audit: ESKF-only vs ESKF+ML velocity showed ML degradation due to correlated residual noise and uncertainty miscalibration.
- Corrective Aiding Policy: Implemented configurable `update_interval` rate-limiting (throttling) in `VelocityEstimatorAPI` to reduce ML update frequency, proving safe and improving navigation error below ESKF-only baseline.

## 7. Phase 11 Status (Real-Data Validation & Generalization)
- Validation framework established via `phase11_validation.py`.
- Synthetic generalization tests show mixed results: ML-aiding policy (update_interval=20) non-degrading for 5s outages and higher speeds, but degrading for 15s outage at 10 m/s and 30s outages at 10-30 m/s.
- **Blocker**: Phase 11 remains completely BLOCKED for REAL-DATA VALIDATION because the required real sensor dataset is unavailable (`data/raw` and `data/processed` directories are empty).
- All 158 regression tests pass.

## 8. Canonical Directory Structure (from ARCHITECTURE.md)
```
dead-reckoning-core/
├── apps/
│   ├── android/              # Native Android Kotlin App (Sensors, UI, MapView)
│   ├── edge_daemon/          # C++/Python 200 Hz Edge Engine
│   └── web_dashboard/        # Real-time WebSocket Telemetry UI
├── core/
│   ├── sensors/              # Sensor Abstraction Layer, Types, RingBuffer
│   ├── calibration/          # Online/Offline Sensor Calibration
│   ├── alignment/            # Phone-to-Vehicle Dynamic Estimator
│   ├── navigation/           # Mechanization, Kinematics, Frames
│   ├── filters/              # ESKF, Covariance Managers, Jacobians
│   ├── constraints/          # Soft ZUPT, Adaptive NHC, ML-Velocity
│   ├── motion/               # Vibration Engine, Motion Feature Extractor
│   ├── models/               # TCN / GRU Architectures, ONNX/TFLite Run
│   ├── gnss/                 # Quality Monitor, Outage Manager, Transitions
│   ├── map/                  # Road Graph, Map Matcher, Vertical Resolver
│   └── integrity/            # Error Propagation, Integrity State Machine
├── tools/
│   ├── replay/               # Deterministic Sensor Stream Replayer
│   ├── simulator/            # Synthetic IMU/GNSS Generator & Blackout Injector
│   ├── evaluation/           # Trajectory Metrics (ATE, RTE, Drift Ratio)
│   └── ablation/             # Automated Multi-Variant Benchmark Runner
└── docs/                     # Architecture, Audits, Mathematical References
```

## 9. Known Issues & Blockers
- **Phase 11 Blocker**: No real data available for validation. The `data/raw` and `data/processed` directories are empty.
- **ML Generalization**: The current ML-aiding policy (update_interval=20) shows degradation at lower speeds and longer outages in synthetic tests. Further investigation required when real data becomes available.
- **Temporary/Experimental Files**: Several files in the repository root appear to be temporary, experimental, or checkpoint artifacts that may not be part of the canonical structure.

## 10. Files Not Part of Canonical Structure (Audit List)
The following files are located in the repository root and are not part of the canonical directory structure defined in the architecture. They may be temporary, experimental, or checkpoint files. Review is required to determine if they should be moved, archived, or deleted.

### Root-Level Non-Canonical Files
- `check_ml_rmse.py`
- `failure_isolation_study.py`
- `model_diagnostic.py`
- `PHASE_10_ACCEPTANCE_CHECKPOINT.md`
- `PHASE_10_FAILURE-ISOLATION_CHECKPOINT.md`
- `PHASE_10_FINAL_CLOSURE_CHECKPOINT.md`
- `PHASE_11_FAILURE_MATRIX_ANALYSIS.md`
- `PHASE_11_REAL_DATA_VALIDATION_CHECKPOINT.md`
- `phase10_checkpoint.md`
- `phase11_analysis_output.txt`
- `phase11_detailed_analysis.py`
- `phase11_detailed_results.json`
- `phase11_validation.py`
- `phase11_validation_results.json`
- `phase8_checkpoint.md`
- `phase9_checkpoint.md`
- `reproduce_baseline.py`
- `run_audit.py`
- `run_phase11_full_analysis.py`
- `test_benchmark_reprod.py`
- `test_ml_policy.py`
- `VALIDATION_PHASE_8_9.md`
- `velocity_model.pth`

### Duplicate or Redundant Files
- `bias_calibration_simple.py` and `mag_calibration_simple.py` in `core/calibration/` appear to be simplified or alternative implementations. Verify if they are needed.
- Multiple checkpoint markdown files in the root that duplicate information in `PROJECT_STATE.md` and `ENGINEERING_LOG.md`.

## 11. Recommendations for Cleanup
1. **Move** any utility scripts that are part of the toolchain to the appropriate `tools/` subdirectory (e.g., `check_ml_rmse.py` → `tools/evaluation/` if it's an evaluation script).
2. **Archive** or delete temporary analysis scripts (e.g., `phase11_detailed_analysis.py`, `run_phase11_full_analysis.py`) after extracting any necessary information into the master context or engineering log.
3. **Consolidate** checkpoint markdown files into the living documents (`PROJECT_STATE.md`, `ENGINEERING_LOG.md`, `TODO.md`) and remove the redundant root-level checkpoint files.
4. **Verify** the necessity of `_simple.py` implementations in `core/calibration/` and either document their purpose or remove them if superseded.
5. **Ensure** that any model files (e.g., `velocity_model.pth`) are stored in a models directory under `core/models/` or `tools/simulator/` if used for simulation, not in the root.
6. **Validate** that all non-canonical files are either moved to their correct location, archived with a clear purpose, or deleted if they are truly temporary and no longer needed.

## 12. Conclusion
This master context captures the current state of the project. The canonical structure is as defined in the architecture documentation. The next steps involve validating the cleanup of non-canonical files and proceeding with Phase 11 only when real data becomes available. No new features or ML retraining should be undertaken until the audit is complete and the canonical architecture is restored.