# Novelty Layer — Locked Specification

Every feature below is IN SCOPE and must be implemented exactly as described.
Nothing outside this file is novelty scope. If an idea isn't here, it doesn't
get built without explicit sign-off (see `CLAUDE.md` rule 1).

---

### N1. Motorcycle lean-compensated Non-Holonomic Constraint (highest-priority, critical path)

**Problem it solves**: standard car NHC assumes the IMU/body frame stays
aligned with the road plane. A leaning motorcycle rotates the body frame
relative to ground during cornering, so naive NHC injects false velocity
corrections while leaning.

**Spec**:
1. Estimate roll/lean angle in real time via an EKF-based lean angle estimator
   (gyro + accel, gravity-vector-based lean inference), not just static roll.
2. Apply the NHC (no lateral slip, no vertical velocity) in the
   **lean-compensated frame** — i.e. rotate the constraint by the estimated
   lean angle before applying it, not in the raw body frame.
3. This only activates when Vehicle-Type Classifier = two-wheeler. Car/truck
   use standard NHC.

**Data reality**: IO-VNBD has no two-wheeler data. This means the lean-NHC
module cannot be validated against IO-VNBD. The plan is:
- Screening round: validate lean-angle estimator logic against a small,
  self-collected motorcycle dataset (phone + reference GPS track on an actual
  ride) OR a synthetically derived lean-dynamics dataset built by applying
  known lean-angle kinematic transforms to car IMU data as a bridge — pick
  ONE approach and record the choice in `docs/OPEN_QUESTIONS.md` before
  starting Phase 5. Do not silently skip validation.
- Do not claim IO-VNBD validates the two-wheeler path in any report or plot.
  Label two-wheeler results explicitly as validated on the separate dataset.

**Mounting assumption**: frame/tank/fixed mount only, not handlebar. Handlebar
mounting conflates steering yaw with vehicle heading yaw and is explicitly
OUT of scope for this project — document this as a stated limitation, do not
attempt to solve steering-yaw separation.

---

### N2. Predictive GNSS outage detection

**Problem it solves**: most systems react to GNSS loss after it happens. This
predicts degrading signal quality a few seconds ahead and pre-emptively shifts
trust toward INS, so the fusion transition is smoother.

**Spec**:
1. Use Android's raw GNSS measurements API (GnssStatus / GnssMeasurements) to
   read per-satellite C/N0 and HDOP.
2. Maintain a short trailing trend (few-second window) of these values.
3. When trend crosses a degrading threshold (falling C/N0 across visible
   satellites, rising HDOP), emit an early "signal degrading" trust signal to
   the Fusion Engine — this shifts the Kalman filter's measurement trust
   gradually, ahead of a hard fix loss, rather than the filter being surprised
   by a sudden dropout.
4. **Platform scope: Android only.** iOS has no equivalent raw GNSS API. Do
   not attempt an iOS version of this specific feature. This is a stated,
   accepted platform limitation — not a bug to fix.

---

### N3. Chi-squared (NIS) innovation gating on every fusion update

**Problem it solves**: naive fusion accepts every GNSS update even when it's
statistically inconsistent with the current state estimate (e.g. multipath in
urban canyons giving a bad-but-not-missing fix), corrupting the filter.

**Spec**:
1. Compute the Normalized Innovation Squared (NIS) for every incoming GNSS
   measurement update.
2. Test against the chi-squared distribution at the correct degrees of
   freedom for the measurement dimension.
3. If the update fails the consistency test, down-weight or reject it rather
   than applying it at full trust.
4. Log NIS pass/fail history — this feeds directly into the confidence
   ellipse (N4) and is required output for the eval/benchmark phase.

---

### N4. Uncertainty-honest UI: growing/shrinking confidence ellipse

**Problem it solves**: most consumer nav UIs render a single dot with false
confidence. Emergency responders and delivery drivers need to know when the
system is guessing.

**Spec**:
1. Render position as a covariance ellipse (from the fusion engine's state
   covariance matrix), not a single point, during GNSS-denied operation.
2. Ellipse visibly grows as INS-only drift accumulates.
3. Ellipse visibly tightens immediately on GNSS reacquisition and a
   NIS-passing update.
4. This is a UI requirement tied directly to N3's covariance output — do not
   implement it as a cosmetic/fake animation; it must reflect the actual
   filter covariance.

---

### N5. Magnetometer disturbance gating

**Problem it solves**: in-vehicle magnetometers are unreliable (steel body,
electronics, alternator) and naive systems trust heading from them anyway.

**Spec**:
1. Continuously monitor magnetic field magnitude and short-term consistency.
2. If field magnitude deviates from expected local reference or shows abrupt
   jumps/instability, flag magnetometer as disturbed.
3. When flagged, drop magnetometer contribution to heading estimation
   entirely for that period — heading falls back to gyro-integrated +
   GNSS-course-derived heading only. Do not blend in an unreliable
   magnetometer at reduced weight; gate it off/on, not a continuous discount.

---

### N6. Vehicle-type auto-classification from vibration signature

**Problem it solves**: the system needs to know car vs truck vs two-wheeler
to select the correct NHC variant (N1) and map-matching profile, without
manual user input.

**Spec**:
1. Short window of accel/gyro vibration spectrum (engine harmonics, chassis
   response) fed to a lightweight classifier (statistical features + small
   classifier head — keep this simple; it is a supporting feature, not a
   headline one, do not over-invest model complexity here).
2. Output feeds N1 (NHC variant selection) and the map-matching module's
   confidence profile.
3. Re-classify periodically (not just once at startup) in case of a
   misclassification, but do not thrash — require sustained-window agreement
   before switching class.

---

### N7. MEMS/FOG shared-framework, swappable-correction model

**Problem it solves**: honestly satisfying "must also work with external IMU
sensors" without falsely claiming one neural network generalizes across
sensor grades it was never trained on.

**Spec**:
1. The classical strapdown mechanization + EKF/UKF fusion backbone
   (`/engine/fusion`) is IDENTICAL for MEMS (mobile) and FOG (edge) paths.
2. The AI correction/noise model is NOT identical:
   - MEMS/mobile path: full AI Speed & Vibration Filter + AI fusion
     correction, trained on IO-VNBD + collected data (this is where the
     drift problem is worst and AI correction earns its value).
   - FOG/edge path: lightweight or classical-only noise model — FOG-grade
     IMUs already have low drift; do not apply the MEMS-tuned AI correction
     to FOG data, since it was trained to undo noise patterns FOG data
     doesn't exhibit.
3. Per-device calibration (bias/scale-factor estimation at startup or during
   a short calibration drive) is used to handle unit-to-unit variance WITHIN
   a sensor grade. It is explicitly NOT a substitute for grade-specific
   training, and must not be described as one in any report.

---

### N8. Per-device calibration (no full retraining)

**Spec**:
1. On first use (or on request), run a short calibration routine (stationary
   + short drive) to estimate this specific device's accel/gyro bias and
   scale-factor offsets.
2. Store per-device calibration parameters locally; apply as a correction
   layer before the AI filters and strapdown mechanization.
3. This does NOT retrain any model weights. Model weights are fixed from
   offline training (see `docs/ARCHITECTURE.md` section 5). Do not implement
   on-device fine-tuning or weight updates anywhere in this project.

---

## Explicitly OUT of scope (do not implement)

- Handlebar-mount steering-yaw separation (N1 mounting note)
- iOS predictive outage detection (N2 platform note)
- On-device model training/fine-tuning of any kind (N8 note)
- Any vehicle CAN-bus/OBD-II data path — the entire premise is phone-only
- Any UI feature beyond the navigation view + confidence ellipse specified
  in N4 (no social features, no route optimization, no traffic overlay, etc.)
