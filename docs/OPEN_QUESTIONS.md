### Question 7: Two-Wheeler Vehicle Classifier Misidentification
- **Status**: **NOT PURSUED — Magnetometer Problem Is Primary**
- Classifier loads and runs correctly (TFLite active)
- Heading observability gap exists regardless of vehicle type classification

---

## Approved Optimization Plan (Phase 13+ Post-Benchmark)

**Decision Date**: 2026-10-22  
**Decision Authority**: Project Owner  
**Scope**: Non-scope-expanding optimization within existing Phase 6/7/13 architecture. No new sensors, no new AI models, no structural changes to the EKF.

### Problem Statement
The Phase 13 benchmark validation shows 5/16 sessions passing the ≤10% official target, but the remaining 11 sessions fail catastrophically (30%–318,575% drift). Root cause is unobservable yaw heading drift during 60s GNSS blackout with consumer MEMS IMUs.

### Approved Fixes (Within Current Architecture)

#### 1. Map-Matching as Live EKF Measurement Update (Phase 7 Integration)
- **File**: `engine/fusion/fusion_engine.py`, `engine/fusion/ekf.py`, `engine/map_matching/`
- **Change**: Move HMM map-matching from post-process to a continuous pseudo-measurement inside the EKF loop during GNSS outage.
- **Mechanism**: When `mode == PURE_DEAD_RECKONING` and map-matching confidence > threshold, inject `update_map_matching_cross_track` and `update_map_matching_heading` as EKF measurement updates.
- **Constraint**: Road geometry provides absolute heading reference, directly solving the observability gap.
- **Scope Compliance**: Uses existing N7/Phase 7 modules; no new algorithm, just tighter integration.

#### 2. Adaptive NHC (Centripetal-Acceleration-Aware)
- **File**: `engine/fusion/ekf.py` (in `update_nhc`), `engine/nhc_zupt/constrained_ins.py`
- **Change**: Scale NHC measurement noise `sigma_nhc_x` proportionally to lateral acceleration magnitude.
- **Mechanism**: `sigma_nhc_x_adaptive = sigma_nhc_x * (1 + k * |a_lat|)` where `a_lat` is lateral acceleration in vehicle frame.
- **Rationale**: During high-G turns, "no lateral slip" is physically violated; tightening the constraint injects false corrections. Loosening it prevents filter divergence.
- **Scope Compliance**: Modifies existing NHC implementation (N1); no new constraint type.

#### 3. Covariance Tuning for Heading State
- **File**: `engine/fusion/ekf.py` (process noise Q, `outage_scale` logic)
- **Change**: 
  - Increase `Q[6:9, 6:9]` (attitude process noise) during turns using gyro magnitude.
  - Replace blunt `outage_scale = 5.0` with state-dependent scaling: `outage_scale_heading = f(gyro_mag, time_since_gnss)`.
- **Rationale**: Current `outage_scale` inflates all states equally. Heading needs more uncertainty growth; velocity less (NHC holds).
- **Scope Compliance**: Parameter tuning within existing ES-EKF framework.

#### 4. AI Speed Filter Gating by Heading Confidence
- **File**: `engine/fusion/fusion_engine.py` (AI speed update section)
- **Change**: Gate continuous AI speed updates by `yaw_var_rad2 < threshold`. If heading uncertainty is high, pause AI speed aiding to prevent divergence along wrong heading.
- **Rationale**: OPEN_QUESTIONS.md documents that continuous AI speed aiding *accelerates* divergence when heading is bad (S1, TW2). This is a safety gate.
- **Scope Compliance**: Uses existing `heading_uncertainty` parameter in `update_ai_forward_speed`; just changes when it's called.

### Validation Plan
- Run `eval/run_full_benchmark.py` on all 16 sessions.
- Target: ≥12/16 sessions passing ≤10% (75% pass rate).
- Stretch: Vw17-level performance (1-2%) on straight segments maintained.

### Rollback Criteria
If any fix degrades S4 (current best at 30%) or Vw17 (previous best at 0.96%), revert immediately.
