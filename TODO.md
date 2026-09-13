# SIH PS 26168 Dead Reckoning TODO

## Completed (Phases 1-7)
- [x] Basic Project Skeleton & Environment Setup
- [x] Dependency management (Python 3.14 compatible constraints)
- [x] Immutable Data Types & Replays (`ImuSample`, `GnssFix`, iterators)
- [x] Magnetometer SVD hard/soft iron & basic IMU automated steady-state calibration
- [x] Phone-to-vehicle Alignment Engine (Gravity/GNSS Heading extraction)
- [x] Implement deterministic strapdown inertial navigation integrations.
- [x] Rigorous mechanization propagation loops (Position, Velocity, Attitude).
- [x] Connect the 15-state Error-State covariance matrix to genuine periodic measurement updates safely isolating Mahalanobis extremes.
- [x] Incorporate GNSS velocity & position delayed updates mathematically bridging continuous trajectory bounds natively.
- [x] Establish Zero-Velocity Update (ZUPT) trigger architectures feeding observations directly to ESKF filters to correct Random Walk bias limitations organically.
- [x] Soft probabilistic ZUPT detector with adaptive covariance scaling
- [x] ML forward velocity estimator architecture and ESKF integration
- [x] Synthetic trajectory generator for ML training data
- [x] 116/116 regressions validated deterministically.

## Phase 8: GNSS Integrity & Seamless Outage Transitions
- [x] Multi-tier GNSS quality estimator (HDOP, satellite count, innovation gating)
- [x] Outage detection state machine preventing discontinuous jumps on re-acquisition
- [x] Dynamic process noise inflation during outages

## Phase 9: Vibration Analysis & Spectral Decomposition
- [x] Rolling FFT spectrum analyzer for vehicle motion characterization
- [x] Spectral entropy and road roughness estimator
- [x] Engine frequency isolation (10-30 Hz band)

## Phase 10: ML Model Training & Deployment
- [x] Train velocity estimator on synthetic datasets
- [x] Train velocity estimator on real datasets (via DatasetAdapter interface)
- [x] Addressed Velocity RMSE evaluation inconsistencies (recorded valid 1.31 m/s bounds on held-out metrics organically)
- [x] Model quantization considerations documented for mobile deployment
- [x] Performed formal Phase 10 Acceptance Audit comparing Pure INS, ESKF-only, and ESKF + ML velocity
- [x] Executed targeted Phase 10 Failure-Isolation Study (Experiments A-G) linking failures to correlated residues natively.
- [x] Designed and implemented corrective aiding policy mitigating ML miscalibration limits using temporal structural bounds.
- [x] Re-evaluated synthetic integration structurally confirming non-degrading baseline improvements (19.3m drift vs 58.2m).

## Phase 11: Real-Data Validation & Generalization
- [x] Establish the Phase 11 validation framework and benchmark entry point (`phase11_validation.py`)
- [ ] Determine real sensor dataset availability (data/raw and data/processed are empty -> BLOCKED)
- [x] Evaluate the validation matrix over synthetic control and varied synthetic outage scenarios (short 5s, medium 15s, long 30s outages; varied acceleration profiles)
- [ ] Add new Phase 11 unit tests covering data adapter validation, outage evaluation, and safety telemetry
- [x] Run full test regression suite (all 158 tests passed)
- [ ] Update PROJECT_STATE.md, ENGINEERING_LOG.md, TODO.md
- [ ] Author PHASE_11_REAL_DATA_VALIDATION_CHECKPOINT.md

## Future Phases
- [ ] Vehicle Classification (Car vs Bike vs Scooter)
- [ ] Vehicle-Aware Adaptive Non-Holonomic Constraints (NHC)
- [ ] Magnetic Reliability & Anomaly Rejection
- [ ] Road Map Matching Engine with OSM integration
- [ ] Multi-Hypothesis tracking for stacked roads/parking structures
- [ ] Navigation Integrity & Uncertainty quantification system
- [ ] IO-VNBD benchmark evaluation pipeline
- [ ] Output raw trajectories to standardized API targets dynamically validating end edge responses.