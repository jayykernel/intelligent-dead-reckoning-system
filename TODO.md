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
- [x] Validated velocity RMSE ~0.96 m/s during simulated outages (synthetic validation)
- [x] Model quantization considerations documented for mobile deployment

## Future Phases
- [ ] Vehicle Classification (Car vs Bike vs Scooter)
- [ ] Vehicle-Aware Adaptive Non-Holonomic Constraints (NHC)
- [ ] Magnetic Reliability & Anomaly Rejection
- [ ] Road Map Matching Engine with OSM integration
- [ ] Multi-Hypothesis tracking for stacked roads/parking structures
- [ ] Navigation Integrity & Uncertainty quantification system
- [ ] IO-VNBD benchmark evaluation pipeline
- [ ] Output raw trajectories to standardized API targets dynamically validating end edge responses.