# Open Questions / Decisions Log

Claude Code must write here — not implement silently — whenever it hits:
- an ambiguous requirement in the phase checklist
- a specified approach that turns out to be infeasible
- a decision the spec explicitly defers to the person (e.g. Phase 1's
  two-wheeler data bridge choice)
- a target that was not met during benchmark validation

Format:

## [Phase N] <short title>
**Date**:
**Issue**:
**Options considered**:
**Decision needed from**: (person) / **Decision made**: (if resolved)
**Resolution**:

---

## [Phase 3] TFLite export infeasibility on Python 3.14
**Date**: 2026-09-17
**Issue**: The Phase 3 exit criteria require a TFLite model export. Originally attempted on Python 3.14.0, where TensorFlow (and its TFLite converter) does not provide stable wheel support, leading to installation failure.
**Options considered**:
  - Option A: Use PyTorch to train and export to ONNX. (Rejected: Architectural translation risk; CNN/GRU architectures convert less reliably through PyTorch→ONNX→TFLite).
  - Option B: Set up a pinned virtual environment (Python 3.11) with a stable TensorFlow release and train natively in TF/Keras.
**Decision made**: Option B.
**Resolution**: The root cause was a Python/TensorFlow version mismatch (an environment problem), not an architectural blocker. Created a pinned Python 3.11 virtual environment under `/training/venv`, installed TensorFlow/tf-keras, and rewrote `train_speed_filter.py` purely in TF/Keras for native TFLite export. ONNX is specifically disallowed.

---

## [Phase 3] Train/test leakage caught and corrected
**Date**: 2026-09-17
**Issue**: Initial Phase 3 drift evaluation was run on session S1 (Driver A), which is in TRAIN_SESSIONS per `training/dataset_splits.py`. The reported 90.7% drift reduction (2148.91% → 199.55%) is in-sample and overstates generalization performance — the model saw this exact session during training.
**Resolution**: Caught before the Screening Package was finalized. Re-ran evaluation on held-out TEST_SESSIONS only:
  - **S4 (Driver A)**: Same driver/vehicle as baseline, genuinely held out. 60s window: 4651.03% → 190.18% drift (95.91% reduction), MAE 3.72 m/s.
  - **Vta26 (Driver E)**: Different driver/vehicle. 60s window: 1518.29% → 204.83% drift (86.51% reduction), MAE 4.79 m/s.
**Decision made**: S1 in-sample numbers are excluded from the Screening Package headline results. Only S4 and Vta26 held-out test results are reported as representative of generalization. S1 numbers retained in internal docs for reference only, clearly labeled "in-sample, not representative."



## [Phase 1] Two-wheeler data bridge approach for N1 validation
**Date**: 2026-09-17
**Issue**: IO-VNBD contains no two-wheeler data. The lean-compensated NHC (N1)
requires a validation dataset. Per `docs/NOVELTY_SPEC.md` N1, one of two
approaches must be chosen before Phase 5.
**Options considered**:
  - Option A: Self-collected motorcycle ride-log (phone + reference GPS) — real
    dynamics, directly validates the lean-angle estimator and lean-compensated
    NHC on actual two-wheeler data.
  - Option B: Synthetic lean-dynamics dataset derived by applying known
    lean-angle kinematic transforms to car IMU data — available immediately,
    but does not capture real two-wheeler vibration, engine harmonics,
    or actual lean maneuvers.
**Decision made**: Option A (self-collected ride-log data) is the primary
validation dataset for the N1 two-wheeler claim. Option B (synthetic
lean-transform of car data) may only be used as an early bootstrap/sanity-check
during development, never as the validation dataset for the N1 claim.
Real two-wheeler data will be added to `data/raw/two_wheeler/` before Phase 5.
**Resolution**: Decided. No action needed until Phase 5; data collection is
in progress independently.
