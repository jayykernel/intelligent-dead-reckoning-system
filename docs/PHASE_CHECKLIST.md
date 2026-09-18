# Phase-by-Phase Execution Checklist

Rules: work top to bottom, one box at a time. Do not check a box until its
exit criteria are actually met and verified. See `CLAUDE.md` for the full
execution protocol and commit convention.

---

### Phase 0 — Repository & Environment Setup
- [x] Repo initialized with the exact folder structure from `CLAUDE.md`
- [x] `README.md` with project one-liner, setup instructions
- [x] Python env + dependency lockfile for `/training` and `/engine`
- [x] Android project skeleton in `/mobile` (empty screens, no logic yet)
- [x] `docs/OPEN_QUESTIONS.md` created (empty, ready for use)
- **Exit criteria**: repo builds/runs empty skeleton on both sides; folder
  structure matches spec exactly.
- **DO NOT**: add any navigation/sensor logic yet. This phase is scaffolding only.

---

### Phase 1 — Data Pipeline
- [x] IO-VNBD downloaded, parsed, documented (format, fields, coordinate frame)
- [x] Preprocessing pipeline: sync/resample IMU streams, unit normalization
- [x] Train/test split defined and fixed (recorded, not re-randomized later)
- [x] Decision recorded in `docs/OPEN_QUESTIONS.md`: which two-wheeler data
  bridge approach was chosen (self-collected vs synthetic), per N1 spec
- **Exit criteria**: a script that loads a raw IO-VNBD session and outputs a
  clean, synced, normalized array, with a sanity-check plot of raw trajectory.
- **DO NOT**: start model training in this phase.

---

### Phase 2 — Classical Strapdown INS Baseline (no AI yet)
- [x] Physics-only strapdown mechanization implemented in `/engine`
- [x] Run on IO-VNBD subset, measure raw drift (no AI, no constraints)
- [x] This becomes the baseline number all later phases are compared against
- **Exit criteria**: drift plot + drift percentage documented for baseline.
  This number should be bad (that's expected — it's the "why we need AI" proof).
- **DO NOT**: add NHC, ZUPT, or AI correction yet — this phase must be pure
  physics so later improvements are measurable against it.

---

### Phase 3 — AI Speed & Vibration Filter (training)
- [x] Model architecture chosen and documented (CNN/GRU per problem statement)
- [x] Trained on IO-VNBD subset in `/training`
- [x] Exported to TFLite, size/latency checked against mobile feasibility
- [x] Inference re-run through `/engine`, drift re-measured vs Phase 2 baseline
- **Exit criteria**: measurable drift improvement over Phase 2 baseline, with
  a position plot, saved model checkpoint, and TFLite export present in
  `/training/models`.
- **DO NOT**: touch map-matching, fusion engine, or mobile app yet.

---

### CHECKPOINT — Screening Package
- [x] `/screening_package` contains: trained preliminary AI model, position
  plot from inference on IO-VNBD subset, short writeup per problem statement
  requirements
- [x] Package reviewed against the original problem statement's screening
  requirement before submission
- **Exit criteria**: package is submission-ready. This is a hard gate — do
  not proceed to Phase 4 until this is done, since screening happens before
  the finale build continues.

---

### Phase 4 — Alignment & Calibration Engine
- [x] Phone-frame → vehicle-frame rotation estimation implemented
- [x] Works for dashboard-mount and mobile-holder cases (per problem statement)
- [x] Per-device calibration routine (N8) implemented
- **Exit criteria**: calibration produces a stable rotation matrix within a
  bounded time window on test data; documented accuracy of alignment.
- **DO NOT**: attempt handlebar/steering-yaw separation (explicitly out of
  scope, N1 note).

---

### Phase 5 — Vehicle-Type Classifier + NHC/ZUPT (incl. lean-compensated)
- [x] Vehicle-type classifier (N6) trained and integrated
- [x] Standard NHC + ZUPT implemented for car/truck
- [x] Lean-angle EKF estimator implemented for two-wheeler
- [x] Lean-compensated NHC (N1) implemented and applied only when classified
  as two-wheeler
- [x] Validated against the two-wheeler data source decided in Phase 1
- **Exit criteria**: drift improvement measured separately for car (IO-VNBD)
  and two-wheeler (chosen bridge dataset) paths, clearly labeled as such.
- **DO NOT**: claim two-wheeler validation on IO-VNBD data anywhere.

---

### Phase 6 — GNSS+INS Fusion Engine (EKF/UKF + AI correction + NIS gating)
- [ ] Core EKF/UKF fusion implemented (classical backbone, per architecture)
- [ ] AI correction module (N7, MEMS path) integrated
- [ ] Chi-squared NIS gating (N3) implemented on every update
- [ ] Magnetometer disturbance gating (N5) implemented
- **Exit criteria**: fusion output drift measured on GNSS-denied simulated
  segments of test data; NIS pass/fail logging functional and inspectable.
- **DO NOT**: implement the FOG/edge correction variant yet — mobile path only.

**NOTE (2026-09-18)**: Phase 6 is documented as a known limitation. Drift target (≤10%) not met due to heading-observability gap during GNSS outage. Full analysis in `docs/OPEN_QUESTIONS.md` and results in `docs/PHASE6_RESULTS.md`. The EKF architecture is sound (proven by S4 partial success at 37.24% drift), but no reliable absolute heading source exists during outage (magnetometer per-session calibration errors 1–140° with ±50–120° noise; GNSS COG unavailable below 1.0 m/s). Phase 6 boxes remain unchecked pending resolution.

---

### Phase 7 — Map-Matching Filter
- [x] OSM offline extract integrated
- [x] HMM-based map matching implemented
- [x] Two-wheeler relaxed-tolerance + no-snap-fallback profile implemented
- **Exit criteria**: matched trajectory visibly snaps to correct road on test
  routes; no-snap fallback demonstrably triggers on a deliberately bad input.

---

### Phase 8 — Predictive Outage Detection
- [ ] Android raw GNSS measurements integration (C/N0, HDOP)
- [ ] Trend detection + early trust-signal emission to fusion engine
- **Exit criteria**: on a test drive/log with a known upcoming outage
  (tunnel/underpass), trust signal visibly shifts before the hard GNSS loss
  timestamp.
- **DO NOT**: attempt this on iOS.

---

### Phase 9 — Seamless Mode Transition Handler
- [ ] State machine for GNSS-aided-INS ↔ pure-INS implemented
- [ ] Transition latency measured against the millisecond budget in
  `docs/BENCHMARKS.md`
- **Exit criteria**: measured transition time meets benchmark; no UI
  freeze/jump at transition in manual test.

---

### Phase 10 — Confidence Ellipse UI
- [ ] Navigation UI renders covariance ellipse tied to real fusion covariance
  (N4), not a cosmetic animation
- [ ] Ellipse grows during INS-only drift, tightens on NIS-passing GNSS update
- **Exit criteria**: visual behavior verified against logged covariance values
  side by side (screenshot/plot pair).

---

### Phase 11 — Mobile App Integration
- [ ] All engine modules wired into `/mobile` via TFLite + shared logic
- [ ] Real-time pipeline running at required update rate (see BENCHMARKS.md)
- [ ] Full navigation UI functional end to end
- **Exit criteria**: live test drive (or recorded playback) runs start to
  finish with no crashes, correct mode switching, ellipse rendering.

---

### Phase 12 — Edge Engine Packaging
- [ ] `/edge` wraps `/engine` with FOG-appropriate correction model (N7)
- [ ] Fixed-frame calibration (no phone-mount assumptions)
- [ ] Output interface for downstream integration (no UI)
- **Exit criteria**: runs against a FOG-rate (or simulated ~200Hz) synthetic
  or available dataset, meets update-rate target in `docs/BENCHMARKS.md`.

---

### Phase 13 — Full Benchmark Validation
- [ ] Dead reckoning drift % measured against official + stretch targets
  (`docs/BENCHMARKS.md`)
- [ ] Fusion update rate measured (mobile 10Hz, edge ~200Hz)
- [ ] Mode-transition latency re-verified end to end
- [ ] All results written to `/eval` with plots, not just numbers
- **Exit criteria**: official benchmarks met at minimum, stretch target result
  reported honestly (pass or not) — no rounding up, no cherry-picked runs.

---

### Phase 14 — Finale Polish & Demo Readiness
- [ ] Demo script / scenario prepared (including a real or simulated GNSS
  blackout to showcase live)
- [ ] Final README, architecture diagram, and results summary finalized
- [ ] Repo history reviewed: every phase has its commit(s), nothing missing
- **Exit criteria**: a person unfamiliar with the repo can clone it, follow
  the README, and reproduce the benchmark results.
