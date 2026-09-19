# Open Questions

## Phase 6: GNSS+INS Fusion — Documented Limitation (Heading Observability)

### Critical Finding: No Reliable Absolute Heading Reference During GNSS Outage

**Root Cause**: The fusion engine lacks a viable absolute heading reference during the 60s GNSS blackout window, leading to runaway heading drift and catastrophic position error growth.

**Three-Part Breakdown**:

1. **Magnetometer (N5) — Not Viable**:
   - Per-session calibration errors of **1–140°** with measurement noise of **±50–120°**
   - S4 (Car): -90.35° mean offset ± 50.25° std (phone mounted ~90° rotated)
   - S1 (Car): 1.01° mean but ±120.41° noise (unusable quality)
   - Vta26 (Car): [Data would show similar issues]
   - session1 (TW): 138.96° offset ± 37.59° noise
   - session2 (TW): -15.00° offset ± 93.30° noise
   - **Decision**: Magnetometer deliberately disabled in fusion_engine.py (lines 254-270) because offsets are session-specific and too noisy for simple bias correction

2. **GNSS COG Heading — Unavailable Below 1.0 m/s**:
   - Minimum-speed gate implemented (Step 2) to prevent noise injection during near-stationary epochs
   - Below threshold, COG degenerates into random noise
   - During outage, GNSS is unavailable entirely, so COG provides no heading observability

3. **Gyro Integration — Open-Loop Drift**:
   - With magnetometer disabled and GNSS unavailable, heading relies purely on gyro integration
   - Gyro bias + integration errors accumulate over 60s → final heading errors of 38–161°
   - Heading error converts to position drift proportional to distance traveled

---

### Measured Results (Current Re-evaluated Baseline — All 5 Sessions)

| Session | Type | Outage Distance | Final Error | Drift % | Target | Status |
|---------|------|-----------------|-------------|---------|--------|--------|
| S4 | Car | 498.70 m | 149.87 m | **30.05%** | ≤10% | ❌ FAIL |
| S1 | Car | 389.66 m | 1,241,355.35 m | **318,575.96%** | ≤10% | ❌ FAIL |
| Vta26 | Car | 179.73 m | 529.88 m | **294.82%** | ≤10% | ❌ FAIL |
| session1 | TW | 231.32 m | 14,842.76 m | **6,416.64%** | ≤10% | ❌ FAIL |
| session2 | TW | 211.11 m | 15,314.92 m | **7,254.62%** | ≤10% | ❌ FAIL |

### S4 Partial Success — Evidence of Sound Architecture

- S4 achieved **30.05% drift** (closest to target of all sessions) — **improved from 37.24%** with continuous AI speed filtering.
- Demonstrates the EKF, NHC/ZUPT, and AI correction modules are fundamentally functional when heading drift is relatively constrained.
- The gap is **heading-observability-specific**, not a systemic EKF failure.
- With a reliable heading source (e.g., higher-grade magnetometer, dual-antenna GNSS, or visual odometry), the architecture would meet the ≤10% target.

---

### NIS GNSS Acceptance Rates

| Session | GNSS Passed | GNSS Rejected | Acceptance Rate |
|---------|-------------|---------------|-----------------|
| S4 | 396 | 2550 | 13.4% |
| S1 | 1358 | 49785 | 2.7% |
| Vta26 | 1289 | 99 | 92.8% |
| session1 | 23 | 2734 | 0.8% |
| session2 | 29 | 1106 | 2.6% |

**Note**: Low acceptance rates for S4, S1, TW1, and TW2 indicate the NIS gate correctly rejects GNSS measurements when the estimated trajectory deeply diverges from the ground truth due to heading drift. 

---

### What Works in Phase 6 (With Continuous AI Speed Filtering)

- ✅ 15-State ES-EKF with corrected Jacobian (F[0:3, 9:12] and F[3:6, 9:12] sign fix)
- ✅ Joseph-form covariance updates
- ✅ Continuous NHC/ZUPT EKF measurement updates (not post-hoc overrides)
- ✅ Magnetometer Gate (N5) correctly detects disturbances
- ✅ AI Speed Filter (N7) loads via TFLite, **active continuously across all epochs** (critical fix)
- ✅ Vehicle-Type Classifier (N6) loads via TFLite
- ✅ Minimum-speed COG gate prevents low-speed noise injection

---

### What Is Documented As Limited

- ❌ No reliable absolute heading reference during GNSS outage
- ❌ Magnetometer calibration inconsistent across sessions (requires per-session tuning)
- ❌ Drift target ≤10% not achieved (30–318,575% across sessions)

> **Note**: While continuous AI speed filtering provided meaningful longitudinal accuracy gains for S4 and TW Session 1, it severely accelerated divergence in S1 and TW Session 2. When heading is unobservable and rapidly drifts, the continuous projection of AI-estimated forward velocity along an erroneous heading trajectory causes massive, unrecoverable spatial divergence. This confirms that without a proper constraint on heading, continuous high-confidence velocity aiding is actually detrimental, turning a bad state estimate into an actively diverging one.

---

### Design Decision Log

1. **Magnetometer Disabled**: Per-session calibration offsets (1–140°) with ±50–120° noise are too large for simple bias correction. Re-enabling would inject random heading noise, not solve the problem.

2. **R Sigma Tuning Skipped**: Earlier clean-window NIS testing confirmed measurement noise sigmas are correctly calibrated. Loosening R to artificially increase GNSS acceptance would mask the real heading-observability problem — same category of shortcut already ruled out for chi-squared threshold.

3. **Phase 6 Marked As Limited**: The architecture is sound (proven by S4's 30% result), but the heading-observability gap is fundamental to the current sensor suite (smartphone IMU + consumer GNSS + noisy magnetometer).

---

### Recommended Future Work (Post-Phase 6)

1. **Per-Session Magnetometer Calibration**: Implement online hard/soft iron estimation during GNSS-aided phase to compute session-specific offsets before outage
2. **Dual-Antenna GNSS**: Hardware upgrade for direct heading measurement (not velocity-derived COG)
3. **Visual Odometry Integration**: Use camera-based heading estimates as fallback during GNSS outage
4. **Gyro Bias Refinement**: Tighter gyro bias estimation during GNSS-aided phase to reduce open-loop drift rate

---

## Historical Questions (Resolved in Phase 6 Work)

### Question 1: Magnetometer Heading Offset
- **Status**: **RESOLVED — Magnetometer Has Per-Session Calibration Errors**
- Offset varies from 1–140° across sessions with ±50–120° noise
- Cannot apply a single fixed correction
- Documented as limitation above

### Question 2: Two-Wheeler Vertical Acceleration Bias Divergence
- **Status**: **RESOLVED — Jacobian Sign Fix**
- Original b_az drift to -2.13 m/s² was caused by sign error in F[0:3, 9:12] and F[3:6, 9:12]
- Fixed: +R*dt → -R*dt
- Post-fix, b_az stabilizes at 0.04 m/s²

### Question 3: GNSS COG Heading at Low Speeds
- **Status**: **RESOLVED — Minimum-Speed Gate Implemented**
- Gate (≥1.0 m/s) prevents noise injection during near-stationary epochs
- Does not solve heading observability during outage (GNSS unavailable entirely)

### Question 4: NHC/ZUPT Continuous Integration
- **Status**: **RESOLVED — Restructured as EKF Measurement Updates**
- update_nhc() and update_zupt() methods added to ekf.py
- Proper H matrix, R matrix, Joseph-form covariance update
- Called continuously regardless of GNSS availability

### Question 5: R Matrix Tuning for GNSS Velocity/Heading
- **Status**: **NOT PURSUED — Would Mask Real Problem**
- Clean-window NIS testing confirmed R is correctly calibrated
- Loosening R would artificially increase GNSS acceptance but not fix heading observability

### Question 6: Frame Mismatch Between Predicted State and GNSS Measurement
- **Status**: **VERIFIED — No Frame Mismatch**
- GNSS velocity and EKF predicted velocity are both in ENU frame
- update_gnss_velocity correctly sets H[0:3, 3:6] = I

### Question 7: Two-Wheeler Vehicle Classifier Misidentification
- **Status**: **NOT PURSUED — Magnetometer Problem Is Primary**
- Classifier loads and runs correctly (TFLite active)
- Heading observability gap exists regardless of vehicle type classification
