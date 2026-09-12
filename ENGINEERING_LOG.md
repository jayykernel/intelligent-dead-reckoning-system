# Engineering Log

**Project:** SIH PS 26168 — AI/ML based Intelligent Dead Reckoning System

---

## 2026-09-10: Phase 0 Complete
- Delivered project audit, system architecture, implementation roadmap, requirements traceability, novelty matrix, and failure analysis.
- Defined 22-phase sequential delivery plan.

## 2026-09-10: Phase 1 Complete
- Created Python project skeleton with `pyproject.toml`, directory structure, `__init__.py` files for all modules.

## 2026-09-10: Phase 2 Complete
- Implemented `core/sensors/data_types.py`: `ImuSample`, `MagSample`, `GnssFix`, `BaroSample`, `SensorType`, `lin_interp`, `slerp`.
- Implemented `core/sensors/synchronizer.py`: Multi-rate sensor synchronizer with closest-match and linear interpolation.
- Implemented `core/sensors/replay.py`: CSV/binary sensor logger and replay iterator.
- Tests: 14 tests passing (data_types: 5, synchronizer: 5, replay: 4).

## 2026-09-10: Phase 3 Complete
- Implemented `core/calibration/bias_calibration.py`: `StaticImuCalibrator`, arbitrary 3D orientation support, explicit `bool()` casting for Python 3.14 compatibility.
- Implemented `core/calibration/mag_calibration.py`: Centered SVD ellipsoid fitting, definiteness polarity alignment, eigenvalue decomposition sqrt matrix.
- Implemented `core/calibration/persistence.py`: `CalibrationProfile`, JSON versioned save/load with proper dataclass reconstruction.
- Implemented `core/calibration/pipeline.py`: `SensorDataPipeline` enforcing Raw → Calibration → Corrected flow.
- Implemented `core/sensors/health.py`: `ImuHealthMonitor`, `MagHealthMonitor`, `SensorHealthManager` with saturation/jitter/variance checks.
- Fixed: NumPy `longdouble` overflow on Python 3.14 via numpy==2.5.3 wheel.
- Fixed: `assert np.True_ is True` failures via explicit `bool()` wrapping.
- Fixed: Ellipsoid fitting sign ambiguity via eigenvalue polarity check.
- Fixed: JSON deserialization type mismatches in `CalibrationProfile.from_dict`.
- Fixed: Magnetometer replay test generating spatially uniform samples (Fibonacci spiral).
- Tests: 32 calibration/health tests + 5 replay determinism tests, 51 total across all modules. 51/51 passing.
- Phase 3 Engineering Checkpoint: APPROVED.

## 2026-09-11: Phase 4 Starting
- Objective: Phone-to-Vehicle Alignment Engine.
- Design based on `docs/ARCHITECTURE.md` Section 4.

## 2026-09-11: Phase 4 Implementation Checkpoint
- **Objective:** Build robust automatic Phone-to-Vehicle Alignment engine mapping Android Sensor Body frame to Forward-Right-Down (FRD) Vehicle frame.
- **Actions:**
  - Implemented strict Hamilton quaternion math utility in `quaternion_utils.py`, eschewing external non-standard library dependencies for core math.
  - Defined rigid Enum-based schemas in `frames.py` covering definitions for Sensor, Body, Vehicle, Earth, and Navigation coordinate systems as well as Alignment states.
  - Implemented `GravityAligner` in `gravity_alignment.py` estimating pure Pitch/Roll via gravity vector isolation during detected zero velocity periods.
  - Implemented `HeadingEstimator` in `heading_estimator.py` providing heading correction mostly via high-speed (>3m/s) GNSS velocity readings while avoiding magnetic interference pitfalls by strictly bounding single magnetic heading contributions.
  - Created composite `AlignmentEngine` linking gravity and heading subsystems. Integrated a State Machine enforcing progression from `UNINITIALIZED` -> `GRAVITY_ONLY` -> `PARTIALLY_ALIGNED`/`FULLY_ALIGNED`.
  - Added robust dynamic reorientation algorithms that compute quaternion delta angles $\Delta\theta > 15^\circ$ switching constraints explicitly to `LOST` state rather than trusting drifted references.
  - Authored comprehensive strict unit testing suite in `tests/core/alignment/` guaranteeing corner case accuracy, Euler boundary wrapping behaviors, deterministic heading/gravity confidence, and multi-state alignment engine state transitions.
- **Status:** Phase 4 successfully unit-tested (75/75 passing overall tests on Python 3.14 / Numpy 2.5.3). Handing off for checkpoint review.

## 2026-09-11: Phase 5 Implementation Checkpoint
- **Objective:** Deterministic Strapdown Inertial Navigation System (INS) Mechanization baseline establishing raw trajectory dynamics from phase 3-4 calibrated outputs.
- **Actions:**
  - Implemented strictly-typed `NavState` recording discrete states linking timestamps, NED Position, Velocity, Vehicle Attitude Quarternions, and 6-DOF dynamic offsets.
  - Implemented `StrapdownINS` utilizing zeroth-order hold kinematics and standard Local Tangent plane mapping via analytic Specific-Force coordinate rotations.
  - Correctly accommodated +Z downwards Navigation (NED) frame $a_n = [f_n \times q] + g_{ned}$ constraints correctly handling $+9.80665 m/s^2$ zero-offset compensation behavior on arbitrary non-leveled tables.
  - Implemented deterministic discrete mathematical transformations for arbitrary IMU input trajectories including circular tests mapping Centripetal force derivations analytically over standard non-holonomic limits.
  - Inscribed a fully analytic 15-state Continuous-Discrete error-state transition matrix ($\Phi_{15x15}$) expanding spatial variables by standard deviation Process Noise representations inside `_propagate_covariance()`.
  - Linked Phase 3, Phase 4, and Phase 5 directly inside `test_integration.py` driving Synthetic Raw Data natively from zero velocity biases into aligned stationary 0-velocity holds without mathematical drift signatures.
- **Status:** Phase 5 successfully unit-tested (84/84 passing overall tests on Python 3.14 / Numpy 2.5.3). Yielding for review prior to Phase 6 Filter propagation.

## 2026-09-11: Phase 6 Implementation Checkpoint
- **Objective:** Design and validate the core Error-State Kalman Filter (ESKF) bridging raw deterministic INS propagation with active stochastic GNSS constraints isolating drifting trajectories.
- **Actions:**
  - Evaluated coordinate error-states defining exactly a 15-state Continuous-Discrete model structure inside `core/filters/eskf.py`. Defined Attitude variations as Multiplicative (`q_true = q_nom * q_err`) mapped across local vehicle limits securely linking linear additive constraints for Position, Velocity, and Biases.
  - Symmetrized Covariance operations explicitly forcing Joseph Formulation `(I - KH)*P*(I - KH)^T + K R K^T` avoiding precision losses propagating numeric unbalances inherently across matrix decompositions. Added numeric fallback checks transitioning cleanly to pseudoinverses upon pseudo-singular anomalies.
  - Configured GNSS measurement frameworks exposing `update_position()`, `update_velocity()`, and localized derivatives like `update_zero_velocity()` safely. Built structural helpers in `earth.py` translating ECEF equivalent LLA maps into explicit navigation grids tracking the NED origin.
  - Wrote fully Synthetic End-To-End verification models mapping intentional deterministic position diversions measuring identical trajectory compensations accurately collapsing pure inertial accumulated bounds down to < 0.05m divergences via ZUPT clamping tracking unmodelled internal hardware biases towards exact 0.5 drift factors.
- **Status:** Phase 6 successfully unit-tested (93/93 passing overall tests on Python 3.14 / Numpy 2.5.3) maintaining explicit frame definitions. Handing off for structural checkpoint analysis.

## 2026-09-11: Phase 7 Implementation Checkpoint
- **Objective:** Implement Motion Intelligence Layer providing intelligent stationary detection (ZUPT) and ML-based forward velocity estimation to aid navigation during GNSS outages.
- **Actions:**
  - Implemented `ZuptDetector` in `core/motion/zupt_detector.py` providing soft probabilistic stationary detection P(stat) ∈ [0,1] based on accelerometer/gyroscope variance, mean deviation from gravity, and temporal consistency smoothing. Supports adaptive measurement covariance scaling: R_zupt = R_base / (P_stat^γ + ε) enabling graceful degradation rather than hard thresholding.
  - Created `VelocityEstimatorAPI` in `core/models/velocity_estimator.py` wrapping a lightweight 1D CNN model (`Velocity1DCNN`) for forward velocity estimation from 6-channel IMU windows (ax, ay, az, gx, gy, gz). Model outputs velocity prediction and log-variance for uncertainty quantification. API provides graceful fallback when PyTorch unavailable or buffer insufficient.
  - Implemented `SyntheticTrajectoryGenerator` in `core/models/dataset_generator.py` for generating ground-truth training data with realistic acceleration profiles (accel/cruise/decel patterns with vibrational noise).
  - Verified ESKF already included `update_forward_velocity()` method from Phase 6, correctly implementing Jacobian H mapping vehicle-frame forward velocity to navigation-frame velocity components with proper attitude coupling.
  - Created training script `tools/train_velocity_model.py` with Gaussian NLL loss for uncertainty-aware training (not executed in this phase due to focus on architecture validation).
- **Test Coverage:**
  - 9 unit tests for ZUPT detector: initialization, stationary detection (P>0.8), moving detection (P<0.3), insufficient data handling, adaptive covariance scaling, temporal consistency, reset, state transitions, high-uncertainty-when-moving.
  - 9 unit tests for ML velocity integration: update acceptance/rejection via Mahalanobis gating, drift correction, high-uncertainty damping, rotation handling, API buffer management, covariance PSD maintenance.
  - 2 integration tests: GNSS outage scenario with ML aiding, traffic stop with ZUPT application.
- **Status:** Phase 7 successfully implemented and validated with 20 new tests (116 total passing). Motion intelligence components ready for integration with constraint engine (Phase 8+). ML model architecture validated; training deferred to Phase 10 with real dataset collection.

## 2026-09-12: Phase 8 Implementation Checkpoint
- **Objective:** Implement GNSS Integrity Pipeline and Seamless Outage Transitions to safely handle GNSS degradation and loss without trajectory discontinuities.
- **Actions:**
  - Implemented `GnssQualityEstimator` computing a continuous [0,1] quality metric from physical attributes (accuracy, satellite count) and temporal/innovation consistency mathematically gated via Mahalanobis residuals, avoiding unplausible trajectory jumps explicitly.
  - Constructed `NavigationModeManager` explicitly defining 6 discrete phases (`GNSS_FIXED, HYBRID_DEGRADED, TRANSITION_TO_DR, DEAD_RECKONING, TRANSITION_TO_GNSS, RECOVERED`) preventing rapid mode chattering via consecutive persistence observation counters.
  - Integrated rigorous Measurement Covariance Scaling mathematically scaling measurement noise inverses against GNSS quality metrics organically forcing ESKF gain decay on compromised constraints rather than discontinuous binary exclusions.
  - Added smooth transition constraints utilizing progressive gain scaling blocks tracking transition epochs across GNSS recovery phases to isolate step-jumps upon rapid coordinate shifts on GNSS reconnection.
  - Implemented `GnssIntegrityPipeline` aggregating subcomponents bridging raw inputs → validity → estimation → filter application logically matching structural design requirements.
- **Test Coverage:**
  - Gnss Integrity components: 8 tests mapping fail-safe traps, Mahalanobis evaluations, kinematic sanity gates, and temporal validations.
  - Navigation mode components: 10 tests confirming explicit transition state boundaries isolating false recoveries from verifiable fix returns.
  - Outage integration schemas: 11 tests mathematically replicating deterministic GNSS disconnect behaviors capturing covariance drift profiles scaling identically to mathematically expected unconstrained inertial accumulations. Discontinuity thresholds verified maintaining baseline jump variances < 5.0m across synthesized multipath anomalies safely rejected against 100m+ naive baselines.
- **Status:** Phase 8 validated successfully. All tests passing deterministically (145/145). Framework guarantees outage persistence correctly managing error-state divergence trajectories seamlessly over temporary sensor failures. Handing off for structural checkpoint analysis.

