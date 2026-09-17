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
