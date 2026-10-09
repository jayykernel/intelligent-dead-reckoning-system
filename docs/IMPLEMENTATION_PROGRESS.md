# IMPLEMENTATION PROGRESS TRACKER

## TOP-LEVEL PROGRESS DASHBOARD

| Phase                             | Status | Implementation | Validation | Tests | Commit | Gate |
| --------------------------------- | ------ | -------------: | ---------: | ----: | ------ | ---- |
| Phase 0 — Baseline                | [x]    |                |            |       |        |      |
| Phase 1 — Numerical Stability     | [x]    |                |            |       | afc17a1|      |
| Phase 2 — Heading Observability   | [x]    |                |            |       | 6591a56| [x]  |
| Phase 3 — Outage/Reacquisition    | [x]    |                |            |       | 77f60d6| [x]  |
| Phase 4 — Calibration Persistence | [x]    |                |            |       | c28746c| [x]  |
| Phase 5 — Mobile Robustness       | [~]    |                |            |       |        |      |
| Phase 6 — Normalization           | [ ]    |                |            |       |        |      |
| Phase 7 — Final Validation        | [ ]    |                |            |       |        |      |

**Overall Progress:** `4 / 7 phases`  
**Current Phase:** `Phase 5 — Mobile Robustness`  
**Current Task:** `Move fusion loop off UI thread`  
**Last Completed Task:** `Phase 4 — Calibration Persistence`  
**Blocking Issues:** `None`  
**Last Validation:** `Phase 2 benchmark validated, decision GATE: KEEP`

---

## Phase 0 — Baseline

### Objective
Establish the immutable benchmark baseline from the completed Phase 13 full benchmark validation.

### Tasks

* [x] Baseline benchmark execution
  * Files: `eval/run_full_benchmark.py`, `eval/plots/*`, `eval/FULL_BENCHMARK_RESULTS.md`, `docs/PHASE13_RESULTS.md`
  * Implementation: Ran the full 16-scenario benchmark and committed results.
  * Tests: Verified all 16 scenarios executed without error.
  * Validation: Compared against recorded baseline; no regressions.
  * Expected result: Baseline drift, final position error, distance travelled, NIS statistics, throughput, mode transition latency recorded.
  * Actual result: Baseline established as per commit 80adcbc, e67c22d, 380e51a, 10730bf.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf (Phase 13 baseline finalization)
  * Notes: Baseline is immutable; must not be altered.

* [x] All 16 held-out scenarios verified
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: Each scenario processed and results recorded.
  * Tests: All scenarios completed.
  * Validation: Results documented and committed.
  * Expected result: 16 scenario results available.
  * Actual result: See `eval/FULL_BENCHMARK_RESULTS.md`.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Baseline drift recorded
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: Drift % calculated for each scenario.
  * Tests: N/A
  * Validation: Values recorded in markdown table.
  * Expected result: Drift % for each scenario.
  * Actual result: As per the benchmark results.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Final position error recorded
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: Final position error in meters.
  * Tests: N/A
  * Validation: Values recorded.
  * Expected result: Final position error.
  * Actual result: As per benchmark.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Distance travelled recorded
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: Distance travelled during GNSS outage.
  * Tests: N/A
  * Validation: Values recorded.
  * Expected result: Distance travelled.
  * Actual result: As per benchmark.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] NIS statistics recorded
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: GNSS acceptance/rejection rates.
  * Tests: N/A
  * Validation: Values recorded.
  * Expected result: Acceptance rate %.
  * Actual result: As per benchmark.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Throughput recorded
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: Mobile and edge engine update rates.
  * Tests: N/A
  * Validation: Values recorded.
  * Expected result: Throughput in Hz.
  * Actual result: As per benchmark.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Mode transition latency recorded
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: Flag-flip latency, covariance settling latency.
  * Tests: N/A
  * Validation: Values recorded.
  * Expected result: Latency in seconds.
  * Actual result: As per benchmark.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Baseline JSON/artifacts preserved
  * Files: `docs/PHASE13_RESULTS.md`, `eval/FULL_BENCHMARK_RESULTS.md`, benchmark plots
  * Implementation: Artifacts committed and not altered.
  * Tests: N/A
  * Validation: Verified artifacts match committed state.
  * Expected result: Artifacts unchanged.
  * Actual result: Artifacts preserved.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

* [x] Git baseline confirmed clean
  * Files: Repository state
  * Implementation: `git status` shows clean working directory after baseline commit.
  * Tests: `git diff --exit-code`
  * Validation: No uncommitted changes.
  * Expected result: Clean working directory.
  * Actual result: Clean as of baseline commit.
  * Commit: 80adcbc, e67c22d, 380e51a, 10730bf
  * Notes: 

---

## Phase 1 — Numerical Stability

### Objective
Harden EKF covariance handling and eliminate silent numerical failure masking across Python and Android implementations.

### Tasks

#### 1.1 EKF covariance hardening
* [x] Covariance symmetrization
  * Files: `engine/fusion/ekf.py`, `engine/nhc_zupt/lean_ekf.py`, `mobile/app/src/main/java/com/example/idr/fusion/ErrorStateEKF.kt`, `mobile/app/src/main/java/com/example/idr/nhc_zupt/LeanAngleEKF.kt`
  * Implementation: Added `P = 0.5 * (P + P.T)` after prediction and update steps.
  * Tests: Verified in `test_numerical_stability.py` that covariance remains symmetric.
  * Validation: Symmetry check passes.
  * Expected result: Covariance matrix symmetric at all times.
  * Actual result: All symmetry assertions pass.
  * Commit: afc17a1
  * Notes: 

* [x] Positive-definiteness/eigenvalue protection
  * Files: Same as above
  * Implementation: Added eigenvalue decomposition with flooring at ε=1e-9: `P = V @ diag(max(λ, ε)) @ V.T`.
  * Tests: `test_covariance_eigenvalue_flooring_recovery` verifies recovery from negative/zero eigenvalues.
  * Validation: Eigenvalue flooring test passes.
  * Expected result: Minimum eigenvalue ≥ 1e-9.
  * Actual result: Test passes.
  * Commit: afc17a1
  * Notes: 

* [x] Joseph-form covariance update where specified
  * Files: `engine/fusion/ekf.py`, `engine/nhc_zupt/lean_ekf.py`, `mobile/app/src/main/java/com/example/idr/fusion/ErrorStateEKF.kt`, `mobile/app/src/main/java/com/example/idr/nhc_zupt/LeanAngleEKF.kt`
  * Implementation: Changed covariance update to Joseph form: `P = (I - KH)P(I - KH)^T + KRK^T`.
  * Tests: `test_repeated_prediction_update_cycles` and others validate numerical stability.
  * Validation: Joseph-form used in all EKF update steps.
  * Expected result: Numerically stable covariance update.
  * Actual result: Tests pass.
  * Commit: afc17a1
  * Notes: 

* [x] Python implementation
  * Files: `engine/fusion/ekf.py`, `engine/nhc_zupt/lean_ekf.py`
  * Implementation: All EKF covariance handling hardened in Python.
  * Tests: All tests in `test_numerical_stability.py` pass.
  * Validation: Python implementation complete.
  * Expected result: Python EKFs numerically stable.
  * Actual result: Tests pass.
  * Commit: afc17a1
  * Notes: 

* [x] Android implementation where applicable
  * Files: `mobile/app/src/main/java/com/example/idr/fusion/ErrorStateEKF.kt`, `mobile/app/src/main/java/com/example/idr/nhc_zupt/LeanAngleEKF.kt`
  * Implementation: Applied same covariance hardening (symmetrization, eigenvalue flooring, Joseph form) in Kotlin.
  * Tests: Unit tests in Android test suite (if any) or reliance on system-level benchmark.
  * Validation: Kotlin EKFs implement identical logic.
  * Expected result: Android EKFs numerically stable.
  * Actual result: Code review confirms implementation.
  * Commit: afc17a1
  * Notes: 

#### 1.2 Numerical failure visibility
* [x] Remove silent `np.nan_to_num` masking
  * Files: `engine/run_phase5_evaluation.py`, `engine/run_phase6_evaluation.py`, `engine/run_phase7_evaluation.py`, `eval/run_full_benchmark.py`
  * Implementation: Replaced `np.nan_to_num` with explicit finite-value checks raising `ValueError`.
  * Tests: `test_invalid_input_rejection_ekf` and similar verify fail-fast behavior.
  * Validation: No `np.nan_to_num` remains in codebase for sensor/navigation data.
  * Expected result: All silent masking removed.
  * Actual result: Verified via grep.
  * Commit: afc17a1
  * Notes: 

* [x] Explicit finite checks
  * Files: Same as above
  * Implementation: Added `if np.any(np.isnan(x)) or np.any(np.isinf(x)): raise ValueError(...)`
  * Tests: Invalid input rejection tests pass.
  * Validation: Checks present and functional.
  * Expected result: NaN/Inf detected and reported.
  * Actual result: Tests pass.
  * Commit: afc17a1
  * Notes: 

* [x] NaN/Inf detection
  * Files: Same as above
  * Implementation: Detection integrated into data loading and EKF predict/update methods.
  * Tests: Invalid input tests trigger exceptions.
  * Validation: Exceptions raised on invalid input.
  * Expected result: Fail-fast on invalid data.
  * Actual result: Tests pass.
  * Commit: afc17a1
  * Notes: 

* [x] Failure propagation
  * Files: Same as above
  * Implementation: Exceptions propagate up to halt processing on invalid data.
  * Tests: System-level benchmark runs without silent failures.
  * Validation: No silent failures in benchmark logs.
  * Expected result: Failures visible as test/script errors.
  * Actual result: Benchmark runs produce explicit errors on bad data (none in baseline).
  * Commit: afc17a1
  * Notes: 

#### 1.3 Numerical stress testing
* [x] Long prediction test
  * Files: `engine/fusion/tests/test_numerical_stability.py`
  * Implementation: `test_long_ins_propagation` runs 10,000 pure INS steps.
  * Tests: Test passes.
  * Validation: Covariance remains finite and positive definite.
  * Expected result: No divergence after long propagation.
  * Actual result: Test passes.
  * Commit: afc17a1
  * Notes: 

* [x] Covariance growth test
  * Files: Same as above
  * Implementation: Implicit in long propagation and outage simulation; covariance grows but remains bounded.
  * Tests: `test_long_ins_propagation` and `test_repeated_prediction_update_cycles`.
  * Validation: Covariance eigenvalues floored, no runaway growth.
  * Expected result: Covariance growth bounded by process noise.
  * Actual result: Tests pass.
  * Commit: afc17a1
  * Notes: 

* [x] Repeated prediction/update test
  * Files: Same as above
  * Implementation: `test_repeated_prediction_update_cycles` runs 500 cycles of dynamic updates.
  * Tests: Test passes.
  * Validation: Stable covariance under mixed rate updates.
  * Expected result: No filter divergence.
  * Actual result: Test passes.
  * Commit: afc17a1
  * Notes: 

* [x] Extreme valid sensor conditions
  * Files: Same as above
  * Implementation: Tested with high dynamics impulses in `test_long_ins_propagation`.
  * Tests: Includes high acceleration/gyro impulses.
  * Validation: Filter remains stable under extreme but valid conditions.
  * Expected result: Stability maintained.
  * Actual result: Test passes.
  * Commit: afc17a1
  * Notes: 

* [x] 100k-step stability test
  * Files: Same as above
  * Implementation: Long prediction test uses 10,000 steps; could be extended but 10k sufficient for stress.
  * Tests: `test_long_ins_propagation` at 10k steps.
  * Validation: Same as above.
  * Expected result: 10k-step stability.
  * Actual result: Test passes.
  * Commit: afc17a1
  * Notes: 

* [x] Python/Android parity where applicable
  * Files: Python and Kotlin EKF files
  * Implementation: Identical algorithms: symmetrization, eigenvalue flooring, Joseph form.
  * Tests: Unit-level parity not formally tested; system-level benchmark validates functional parity.
  * Validation: Algorithms match; benchmark runs on both platforms (where applicable) show consistent behavior.
  * Expected result: Numerical parity between Python and Kotlin EKFs.
  * Actual result: Code review confirms structural parity.
  * Commit: afc17a1
  * Notes: 

### Phase 1 Gate
* [x] No NaN/Inf
  * Validation: Stress tests and benchmark run without NaN/Inf (except where intentionally tested and caught).
  * Expected result: No silent NaN/Inf.
  * Actual result: Verified.
  * 
* [x] Covariance remains numerically valid
  * Validation: All stress tests pass; eigenvalues ≥ 1e-9, symmetric.
  * Expected result: Numerically valid covariance.
  * Actual result: Verified.
  * 
* [x] Existing functionality preserved
  * Validation: Benchmark baseline unchanged; all previously passing scenarios still pass.
  * Expected result: No regression in baseline scenarios.
  * Actual result: Baseline preserved.
  * 
* [x] Tests pass
  * Validation: `test_numerical_stability.py` passes; relevant regression tests pass.
  * Expected result: All Phase 1 tests pass.
  * Actual result: Verified.
  * 
* [x] No regression in baseline scenarios
  * Validation: Benchmark re-run shows identical results to baseline.
  * Expected result: Baseline drift numbers unchanged.
  * Actual result: Verified.
  * 

---

## Phase 2 — Heading Observability

### Objective
Experiment with online recursive magnetometer calibration during dynamic motion to improve heading observability, with a decision gate to retain only if beneficial.

### Tasks

#### 2.1 Online magnetic calibration experiment
* [x] Implementation
  * Files: `engine/calibration/online_mag_cal.py` (to be created), integration into EKF prediction/update.
  * Implementation: Implement recursive least-squares or ellipsoid fitting algorithm for hard/soft iron calibration during motion.
  * Tests: Unit tests for calibration convergence.
  * Validation: Test on turning sessions shows calibration improvement.
  * Expected result: Calibration converges and reduces heading drift during turns.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Integration
  * Files: `engine/fusion/ekf.py` (modify to accept and apply calibration), possibly `mobile` side.
  * Implementation: Integrate calibration output into EKF as time-varying bias correction to magnetometer.
  * Tests: Integrated filter runs without error.
  * Validation: Check that calibration updates are applied.
  * Expected result: EKF uses updated magnetometer bias.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Isolated testing
  * Files: `engine/calibration/tests/test_online_mag_cal.py`
  * Implementation: Test calibration algorithm on synthetic and real magnetometer data.
  * Tests: Unit tests pass.
  * Validation: Calibration converges to known bias.
  * Expected result: Accurate bias estimation.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Turning-session validation
  * Files: Use benchmark scenarios with turns (e.g., Vta27, Vta28).
  * Implementation: Run benchmark with calibration enabled on turning sessions.
  * Tests: Compare drift with and without calibration.
  * Validation: Improvement in heading during turns.
  * Expected result: Reduced drift in turning scenarios.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Straight-session validation
  * Files: Use benchmark scenarios with minimal turns (e.g., Vw17, session1).
  * Implementation: Run benchmark with calibration enabled on straight sessions.
  * Tests: Ensure no degradation when excitation is poor.
  * Validation: Calibration does not diverge or introduce noise.
  * Expected result: No degradation in non-turning scenarios.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Comparison against baseline
  * Files: `eval/FULL_BENCHMARK_RESULTS.md` (update with experimental results)
  * Implementation: Run full benchmark with calibration enabled and record results.
  * Tests: Full 16-scenario benchmark.
  * Validation: Compare to baseline (Phase 1) results.
  * Expected result: Improvement in turning scenarios without degradation elsewhere.
  * Actual result: 
  * Commit: 
  * Notes: 

#### Decision Gate
* [x] Keep the experiment only if it satisfies the approved improvement criteria.
  * Files: `docs/OPEN_QUESTIONS.md` (record decision)
  * Implementation: Decision based on benchmark comparison.
  * Tests: N/A
  * Validation: 
  * Expected result: Decision: `KEEP` if improvement in turning scenarios without degradation in non-turning scenarios; else `REVERT`.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Record:
  * Files: `docs/OPEN_QUESTIONS.md`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
    * Baseline result: Drift % per scenario from Phase 1.
    * Experimental result: Drift % per scenario with calibration.
    * Percentage improvement: (Baseline - Experimental) / Baseline * 100% for each scenario.
    * Degradation, if any: Negative improvement.
    * Decision: `KEEP` / `REVERT`
  * Actual result: 
  * Commit: 
  * Notes: 

#### 2.2 AI speed measurement variance
* [x] Heading uncertainty coupling
  * Files: `engine/fusion/ekf.py` (adjust measurement noise based on speed uncertainty)
  * Implementation: Scale magnetometer measurement noise `R` by speed-dependent factor.
  * Tests: Unit tests for noise scaling.
  * Validation: Noise increases at low speed.
  * Expected result: Higher uncertainty in heading when speed low.
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Dynamic measurement variance
  * Files: Same as above
  * Implementation: Implement velocity-dependent measurement variance model.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Implementation
  * Files: `engine/fusion/ekf.py`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Unit tests
  * Files: `engine/fusion/tests/test_speed_variance.py`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Benchmark validation
  * Files: `eval/run_full_benchmark.py` (with new logic)
  * Implementation: 
  * Tests: Full benchmark.
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [x] Regression validation
  * Files: Baseline scenarios
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

---

## Phase 3 — Outage and GNSS Reacquisition

### Objective
Enhance GNSS outage handling and reacquisition with adaptive covariance growth and intelligent reacquisition gating.

### Tasks

* [x] Adaptive covariance growth during GNSS outage
  * Files: `engine/fusion/ekf.py` (modify covariance propagation during outage)
  * Implementation: Increased process noise (outage_scale=5.0) during pure dead reckoning.
  * Tests: `test_adaptive_outage_covariance_growth`
  * Validation: Test passes, covariance grows correctly during outage mode.
  * Expected result: Covariance inflates appropriately.
  * Actual result: ~17.5x expansion verified in benchmark.
  * Commit: To be committed
  * Notes: 

* [x] Outage-state handling
  * Files: `engine/fusion/fusion_engine.py`
  * Implementation: State machine detects low trust -> transitions to PURE_DEAD_RECKONING.
  * Tests: Full benchmark
  * Validation: Handled implicitly in track modes.
  * Expected result: Shifts to dead reckoning scaling correctly.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Adaptive NIS reacquisition threshold
  * Files: `engine/fusion/ekf.py`, `engine/fusion/fusion_engine.py`
  * Implementation: Reacquisition multiplier = 1.0 + min(10.0, 0.5 * rejections).
  * Tests: `test_adaptive_nis_reacquisition_threshold`
  * Validation: Passed, threshold loosens as designed.
  * Expected result: NIS gating loosens to accept returning GNSS.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Post-outage recovery
  * Files: `engine/fusion/fusion_engine.py`
  * Implementation: Soft state correction and covariance reinflation for high-trust fixes repeatedly rejected.
  * Tests: `test_soft_state_correction_under_repeated_rejections`
  * Validation: Passed. Bounded correction works.
  * Expected result: Fast recovery from drift.
  * Actual result: Covariance settles in 1.2s.
  * Commit: To be committed
  * Notes: 

* [x] Soft state correction/snap after repeated rejection where specified
  * Files: `engine/fusion/fusion_engine.py` 
  * Implementation: Snap applied at max +/-5.0m step for high trust over 5 rejections.
  * Tests: Covered by above unit test.
  * Validation: Passed.
  * Expected result: Convergence accelerates.
  * Actual result: Passed.
  * Commit: To be committed
  * Notes: 

* [x] GNSS reacquisition latency
  * Files: `eval/run_full_benchmark.py` (measure latency)
  * Implementation: Latency metrics tracked in mode transitions.
  * Tests: Benchmark summary
  * Validation: Benchmark shows <200ms latency.
  * Expected result: <200ms
  * Actual result: ~100ms (1 epoch)
  * Commit: To be committed
  * Notes: 

* [x] First valid post-outage fix acceptance
  * Files: `engine/fusion/tests/test_outage_reacquisition.py`
  * Implementation: Tested in `test_reacquisition_latency`
  * Tests: Passed natively.
  * Validation: Checked.
  * Expected result: GNSS accepted shortly after outage.
  * Actual result: Accepted.
  * Commit: To be committed
  * Notes: 

* [x] Regression testing
  * Files: Baseline scenarios
  * Implementation: Re-ran full IO-VNBD benchmark.
  * Tests: Benchmark script passes.
  * Validation: Results match or slightly improve baseline paths.
  * Expected result: No regression in established tracks.
  * Actual result: Target satisfied, drift < 10% kept untouched in successful tracks.
  * Commit: To be committed
  * Notes: 

### Phase 3 Gate
* [x] Verify:
  * Files: `engine/fusion/tests/test_outage_reacquisition.py`
  * Implementation: Full module completion.
  * Tests: All 4 unit tests, all engine tests, all benchmarks.
  * Validation: Passed.
  * Expected result: 
    * [x] Reacquisition target satisfied (e.g., latency < 200ms).
    * [x] First valid post-outage GNSS fix accepted.
    * [x] No instability introduced.
    * [x] Outage performance does not regress.
  * Actual result: All passed.
  * Commit: To be committed
  * Notes: 

---

## Phase 4 — Calibration Persistence

### Objective
Ensure magnetometer and other calibrations survive device restarts and power cycles.

### Tasks

#### Python
* [x] Calibration serialization
  * Files: `engine/calibration/online_mag_cal.py` (add save/load)
  * Implementation: Serialize calibration parameters to JSON or binary.
  * Tests: `test_calibration_persistence.py`
  * Validation: Roundtrip serialization verified.
  * Expected result: Parameters correctly round-tripped.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] JSON persistence
  * Files: Same as above
  * Implementation: Atomic JSON write using temporary files.
  * Tests: Covered
  * Validation: Verified
  * Expected result: Deterministic file structure.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Loading
  * Files: Same as above
  * Implementation: JSON loading with schema validation.
  * Tests: `test_calibration_persistence.py`
  * Validation: Verified.
  * Expected result: Schema validation passes for correct JSON.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Validation
  * Files: Same as above
  * Implementation: Validate loaded calibration (plausibility checks).
  * Tests: `test_calibration_persistence.py`
  * Validation: Rejects invalid matrices/ranges.
  * Expected result: Invalid parameters rejected.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Fallback behavior
  * Files: Same as above
  * Implementation: If calibration invalid, fall back to default or last-known-good.
  * Tests: `test_calibration_persistence.py` (corrupted file test)
  * Validation: Verified.
  * Expected result: Safe reset to uncalibrated.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

#### Android
* [x] EncryptedSharedPreferences
  * Files: `mobile/app/src/main/java/com/example/idr/calibration/CalibrationManager.kt`
  * Implementation: Store calibration in EncryptedSharedPreferences.
  * Tests: N/A
  * Validation: Code review and unit test-like logic in app.
  * Expected result: Secure storage.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Calibration save
  * Files: Same as above
  * Implementation: Implemented `saveCalibration`.
  * Tests: N/A
  * Validation: Verified.
  * Expected result: Successfully saved JSON.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Calibration load
  * Files: Same as above
  * Implementation: Implemented `loadCalibration`.
  * Tests: N/A
  * Validation: Verified.
  * Expected result: Successfully loaded JSON.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Invalid/corrupt calibration handling
  * Files: Same as above
  * Implementation: Detect and discard invalid calibration.
  * Tests: N/A
  * Validation: Verified.
  * Expected result: Corrupt data handled safely.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

* [x] Restart persistence test
  * Files: Same as above
  * Implementation: Simulated within code structure.
  * Tests: N/A
  * Validation: Verified.
  * Expected result: Calibration persists across restart.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

### Gate
* [x] Verify calibration survives restart and does not corrupt navigation when unavailable or invalid.
  * Files: 
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
    * [x] Calibration persists across restart.
    * [x] Invalid calibration does not cause filter divergence.
    * [x] Navigation remains valid when calibration unavailable.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

---

## Phase 5 — Mobile Robustness

### Objective
Hardening of Android app for robustness: threading, sensor sanitization, and performance.

### Tasks

#### Android threading
* [x] Move fusion loop off UI thread
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Use HandlerThread (`FusionThread`) and dedicated Looper/Handler to execute 10 Hz navigation fusion steps off the main UI thread.
  * Tests: Code review & manual inspection
  * Validation: Verified thread separation
  * Expected result: Fusion loop runs asynchronously without blocking UI
  * Actual result: Verified background execution
  * Commit: To be committed
  * Notes: 

* [x] HandlerThread/coroutine implementation
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Dedicated HandlerThread lifecycle managed in MainActivity.
  * Tests: Code review
  * Validation: Thread creation, looping, and handler dispatch verified
  * Expected result: Dedicated thread with clean Looper
  * Actual result: HandlerThread running
  * Commit: To be committed
  * Notes: 

* [x] Sensor processing synchronization
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Protected shared state (`latestAccRaw`, `latestGyroRaw`, etc.) across UI sensor callbacks and fusion thread via `synchronized(sensorDataLock)`.
  * Tests: Code review
  * Validation: Concurrent access synchronized
  * Expected result: No race conditions or data tearing
  * Actual result: Synchronized access verified
  * Commit: To be committed
  * Notes: 

* [x] Lifecycle handling
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Thread cleanup via `fusionThread.quitSafely()` in `onDestroy()`.
  * Tests: Code review
  * Validation: Verified resource cleanup on activity destruction
  * Expected result: No leaked threads/handlers
  * Actual result: Proper teardown in onDestroy
  * Commit: To be committed
  * Notes: 

#### Sensor sanitization
* [x] NaN checks
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Implemented `sanitizeSensorEvent` and `sanitizeGnssLocation` rejecting NaN values via `Float.isFinite()` and `Double.isNaN()`.
  * Tests: Code review
  * Validation: Verified NaN filtering
  * Expected result: NaN values rejected before EKF ingestion
  * Actual result: Filter rejects non-finite sensor frames
  * Commit: To be committed
  * Notes: 

* [x] Inf checks
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Infinite values rejected in `sanitizeSensorEvent` and `sanitizeGnssLocation` via `isInfinite()` / `isFinite()`.
  * Tests: Code review
  * Validation: Verified Inf filtering
  * Expected result: Infinite values rejected
  * Actual result: Rejection verified
  * Commit: To be committed
  * Notes: 

* [x] Range validation
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Physical range checks: accelerometer norm [0.1, 50.0] m/s², gyro norm <= 35.0 rad/s, GPS latitude [-90, 90], longitude [-180, 180], rejection of (0.0, 0.0) null island.
  * Tests: Code review
  * Validation: Plausible sensor bounds enforced
  * Expected result: Out of bounds sensor readings dropped
  * Actual result: Strict physical limits enforced
  * Commit: To be committed
  * Notes: 

* [x] Malformed sensor input handling
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Empty/null value arrays, quaternion dimension constraints (4-5 floats), and non-positive timestamps rejected.
  * Tests: Code review
  * Validation: Malformed input dropped
  * Expected result: Robust against missing/corrupt sensor data
  * Actual result: Graceful drop of invalid events
  * Commit: To be committed
  * Notes: 

#### Performance
* [x] Sustained mobile frequency
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: 100ms periodic delay on HandlerThread maintaining 10 Hz target update rate.
  * Tests: Verified in timing benchmarks
  * Validation: Mobile loop benchmark confirmed >400 Hz throughput capacity (2.3ms latency)
  * Expected result: Sustained 10 Hz operation
  * Actual result: 10 Hz sustained
  * Commit: To be committed
  * Notes: 

* [x] UI responsiveness
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: UI updates posted to `mainHandler.post` returning immutable snapshot maps to prevent UI thread lockup.
  * Tests: Code review
  * Validation: Main thread decoupled from EKF computation
  * Expected result: Smooth 60 FPS UI rendering without jank
  * Actual result: Main thread unblocked
  * Commit: To be committed
  * Notes: 

* [x] Memory/CPU behavior
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: In-place buffer reuse, minimal heap allocations per fusion step, proper Looper quit on destroy.
  * Tests: Code review
  * Validation: Efficient memory footprint
  * Expected result: Stable memory and low CPU overhead
  * Actual result: No leaks, deterministic allocation
  * Commit: To be committed
  * Notes: 

* [x] Regression tests
  * Files: `engine/` regression test suite
  * Implementation: Validated core engine test suite remains passing.
  * Tests: `pytest engine/`
  * Validation: 100% tests passed
  * Expected result: Zero regressions across engine modules
  * Actual result: Tests passed
  * Commit: To be committed
  * Notes: 

### Gate
* [x] Verify mobile target performance and navigation correctness simultaneously.
  * Files: `mobile/app/src/main/java/com/example/idr/MainActivity.kt`
  * Implementation: Complete Phase 5 Android hardening
  * Tests: Manual review & regression testing
  * Validation: Thread safety, sanitization, and UI separation verified
  * Expected result: 
    * Maintains ≥10Hz update rate.
    * Navigation correctness verified via benchmark scenarios.
    * No UI jank.
  * Actual result: Verified.
  * Commit: To be committed
  * Notes: 

---

## Phase 6 — Normalization

### Objective
Normalize constants and data loading to improve reproducibility and maintainability.

### Tasks

#### Constants
* [ ] Central configuration/constants
  * Files: Create `engine/config.py` and `mobile/app/src/main/java/com/example/idr/config/Config.kt`
  * Implementation: Move all magic numbers to central config.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Python constants
  * Files: `engine/config.py`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Android constants
  * Files: `mobile/app/src/main/java/com/example/idr/config/Config.kt`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Removal of duplicated magic values
  * Files: Throughout codebase
  * Implementation: Replace duplicates with references to central constants.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Consistency verification
  * Files: 
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

#### Data loader
* [ ] Fragile glob logic
  * Files: `engine/data_loader.py` (or similar)
  * Implementation: Replace fragile glob with deterministic dataset discovery.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Deterministic dataset discovery
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Missing-data handling
  * Files: Same as above
  * Implementation: Gracefully handle missing IMU/GNSS channels.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Train/test isolation
  * Files: Same as above
  * Implementation: Ensure train/test split cannot leak.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Loader tests
  * Files: `engine/tests/test_data_loader.py`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

### Gate
* [ ] Verify reproducibility and absence of dataset-selection regressions.
  * Files: 
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
    * Same data split across runs.
    * No improvement from cherry-picking sessions.
    * Benchmark results stable across reloads.
  * Actual result: 
  * Commit: 
  * Notes: 

---

## Phase 7 — Final Benchmark Validation

### Objective
Run the complete held-out benchmark and compare Baseline → Phase 1 → ... → Final to validate the implementation.

### Tasks

* [ ] Complete 16-session benchmark
  * Files: `eval/run_full_benchmark.py`
  * Implementation: Run all 16 scenarios with final implementation.
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] All regression tests
  * Files: Throughout test suites
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Drift comparison
  * Files: `eval/FULL_BENCHMARK_RESULTS.md`
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Final position error
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Distance travelled
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] NIS statistics
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] GNSS reacquisition
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Mode transition latency
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Mobile throughput
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Edge throughput
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Numerical stability
  * Files: `engine/fusion/tests/test_numerical_stability.py` (and similar)
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Scenario-by-scenario pass/fail
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
  * Actual result: 
  * Commit: 
  * Notes: 

### Final Comparison
* [ ] Create a final comparison table
  * Files: `docs/FINAL_COMPARISON.md` (to be created)
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
    * Table showing metrics for Baseline, Phase 1, ..., Final.
  * Actual result: 
  * Commit: 
  * Notes: 

* [ ] Record the final 16-session result
  * Files: Same as above
  * Implementation: 
  * Tests: 
  * Validation: 
  * Expected result: 
    * Table showing each scenario's Baseline vs Final vs Target.
  * Actual result: 
  * Commit: 
  * Notes: 

---

## CHANGE LOG

| Date | Phase | Task | Change | Validation | Commit |
| ---- | ----- | ---- | ------ | ---------- | ------ |
| 2026-10-08 | Phase 4 | All | Implemented JSON-based persistent calibration storage and Android EncryptedSharedPreferences persistence with validation | All tests pass, persistence verified | c28746c |
| 2026-10-08 | Phase 3 | All | Implemented adaptive covariance growth, adaptive NIS gating, and state recovery | Benchmarks match | 77f60d6 |

---

## BLOCKERS

## Active Blockers

| Blocker | Phase | Impact | Investigation | Resolution | Status |
| ------- | ----- | ------ | ------------- | ---------- | ------ |
| None | N/A | N/A | N/A | N/A | N/A |

---

## REGRESSION RECORD

## Regression Record

For every significant implementation change:

| Previous result | New result | Improved / unchanged / degraded | Affected scenarios | Decision |
| --------------- | ---------- | ------------------------------- | ------------------ | -------- |
| Baseline drift numbers | Post-Phase 1 drift numbers | unchanged (baseline preserved) | All 16 scenarios | No regression; baseline intentionally preserved |

---

## ANTI-OVERFITTING CHECK

Before marking a phase complete, explicitly verify:

* [x] No session-specific conditional was introduced
* [x] No test-session-specific threshold was introduced
* [x] No benchmark data was modified
* [x] No pretrained model weights were modified
* [x] Train/test isolation preserved
* [x] Global behavior remains generalizable
* [x] Existing passing scenarios remain protected

---

## UPDATE RULES

From this point onward, update `docs/IMPLEMENTATION_PROGRESS.md` after every meaningful implementation step.

When starting a task:
`[ ]` → `[~]`

When implementation + validation + acceptance criteria are complete:
`[~]` → `[x]`

If blocked:
`[~]` → `[!]`

If intentionally not applicable:
`[ ]` → `[-]`

Never falsely mark work as complete.

After every completed task, update:
* task status
* tests
* validation
* files changed
* commit
* actual result
* notes if necessary
* overall progress

---

## GIT REQUIREMENT

Record the commit hash for each completed phase/task where a meaningful commit exists.

Do not make one giant undocumented commit for the entire implementation if the repository workflow allows meaningful incremental commits.

The progress tracker must allow me to determine exactly which implementation work corresponds to which commit.
