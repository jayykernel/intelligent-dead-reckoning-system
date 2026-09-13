# CLAUDE.md — Dead Reckoning Project Instructions

## Project Overview

SIH PS 26168 — AI/ML-augmented Intelligent Dead Reckoning Navigation System.

Physics-informed, machine-learning-augmented inertial navigation system designed for seamless GNSS-denied/degraded vehicle navigation on smartphones (~10 Hz) with optional higher-rate edge hardware (~200 Hz).

## SOURCE OF TRUTH

Before modifying code, ALWAYS read `PROJECT_MASTER_CONTEXT.md` first.

1. PROJECT_MASTER_CONTEXT.md
2. PROJECT_STATE.md
3. docs/ARCHITECTURE.md
4. This CLAUDE.md

The actual repository implementation and tests are authoritative for what currently exists.
Historical checkpoint documents are evidence only; they do not define architecture.

## GOLDEN FOUNDATION — PHASES 1–6

Phases 1–6 are the frozen architectural foundation.

1. Project Foundation
2. Sensor Abstraction & Synchronization
3. Sensor Calibration & Sensor Health
4. Phone-to-Vehicle Alignment
5. Deterministic INS Mechanization
6. Error-State Kalman Filter

Later phases MUST integrate with these canonical interfaces.

Do not redesign, bypass, duplicate, or silently replace Phase 1–6 components. These are frozen architectural contracts.

Any necessary architectural change affecting Phase 1–6 must be explicitly documented, justified, tested, and approved before implementation.

## EXACT DEVELOPMENT ROADMAP

The authoritative roadmap is:

1. Project Foundation
2. Sensor Abstraction & Synchronization
3. Sensor Calibration
4. Phone-to-Vehicle Alignment
5. Deterministic INS Mechanization
6. Error-State Kalman Filter
7. Motion Intelligence / Forward Velocity
8. GNSS Integrity & Seamless GNSS↔DR Transitions
9. Vibration Intelligence
10. ML Velocity Model Development & Validation
11. Real-Data / Generalization Validation
12. Vehicle Classification
13. Vehicle-Aware Motion Constraints
14. Dynamic Bias / Error Adaptation
15. Map Matching
16. Multi-Hypothesis Navigation
17. Parking / Flyover / Level Disambiguation
18. Navigation Integrity & Uncertainty
19. Mobile / Edge Optimization
20. Cross-Device / Cross-Vehicle Robustness
21. Complete System Validation & SIH Benchmark

Never compress these into a generic "Phase 12–21" or "Phase 12–22".

## STRICT PHASE GATING

Never begin a subsequent phase until the current phase has been:

- implemented
- unit tested where applicable
- integration tested where applicable
- validated at the evidence level required by that phase
- documented
- explicitly approved to proceed

Passing pytest alone does NOT constitute full phase validation.

At the end of every phase, STOP and report the checkpoint.
Do not automatically begin the next phase.

Every 3 phases require an integration validation gate.

## EVIDENCE DISCIPLINE

Every claim must identify its evidence level:

- IMPLEMENTED
- UNIT TESTED
- INTEGRATION TESTED
- SYNTHETICALLY VALIDATED
- REAL-DATA VALIDATED
- NOT VALIDATED

Never describe synthetic results as real-world validation.

Never fabricate:
- accuracy
- drift
- benchmark results
- latency
- power
- robustness
- generalization

Measured results must be reproducible from repository code and recorded experiments.

## NAVIGATION ARCHITECTURE

Canonical flow:

Raw Sensors
→ Sensor Abstraction / Synchronization
→ Calibration
→ Phone-to-Vehicle Alignment
→ Motion Intelligence
→ Deterministic INS Mechanization
→ ESKF
→ Adaptive Constraints / Learned Aiding
→ GNSS Integrity & Seamless Transitions
→ Map Matching
→ Multi-Hypothesis / Vertical Resolution
→ Navigation Integrity / Uncertainty
→ Output

The deterministic ESKF remains the navigation backbone.

ML is an aiding component, not the navigation authority.

ML MUST NOT directly overwrite the navigation state.

ESKF-only fallback MUST remain operational.

ML-derived measurements must have:
- confidence
- uncertainty
- gating
- rate limiting where required
- safe fallback behavior

## SENSOR / FRAME RULES

All sensor quantities MUST have explicit:

- coordinate frame
- units
- timestamp semantics
- sampling-rate assumptions

Never silently assume:
- phone axes = vehicle axes
- phone axes = navigation axes
- Z-axis = gravity
- fixed sampling rate
- GNSS availability
- magnetometer reliability

Coordinate conventions must remain consistent with docs/ARCHITECTURE.md.

## COMPLEXITY RULE

Internal complexity is allowed only when it solves a measurable navigation problem.

Do not add:
- models
- filters
- abstractions
- fusion layers
- heuristics
- dashboards
- scripts

merely because they appear sophisticated.

Each new component must state:
1. what failure/problem it addresses
2. why simpler logic is insufficient
3. what measurable metric should improve
4. how it will be validated

## REPOSITORY HYGIENE

Maintain one canonical implementation per responsibility.

Do not introduce duplicate production implementations:
- *_simple.py
- *_v2.py
- *_new.py
- *_experimental.py

Temporary experiments belong under the appropriate tools/ or checkpoints/ location and must not become accidental production architecture.

Keep the repository organized according to docs/ARCHITECTURE.md.

## DEVELOPMENT & TEST COMMANDS

Run all tests:

pytest

Run a specific test:

pytest tests/path/to/test_file.py -v

Python:
3.14

NumPy:
2.x

Use strict typing, dataclasses, and explicit NumPy semantics.

When comparing NumPy boolean results in tests, explicitly cast where appropriate:

bool(...)

## VALIDATION REQUIREMENT

Before declaring a phase complete, record:

- implementation changes
- tests executed
- exact test count
- validation scenarios
- metrics
- known limitations
- blockers
- evidence level
- reproducibility command

Never hide failures merely because the implementation otherwise works.

A failure that exposes a real architectural limitation must be documented and investigated rather than suppressed.

## DOCUMENTATION

Canonical documents:

- PROJECT_MASTER_CONTEXT.md — complete project truth/state
- PROJECT_STATE.md — concise current state
- CLAUDE.md — execution rules for future Claude sessions
- docs/ARCHITECTURE.md — canonical architecture
- VALIDATION_STATUS.md — validation evidence matrix

Historical documents/checkpoints are evidence and history only.

## CURRENT PROJECT GATE

Phase 11 is currently BLOCKED FOR REAL-DATA VALIDATION because the required real sensor dataset is unavailable.

Do NOT:
- start Phase 12
- retrain the ML model
- claim real-world validation
- claim universal ML benefit

until the current Phase 11 gate has been properly resolved.

## FINAL RULE

When uncertain, prefer:

correctness over novelty,
reproducibility over speed,
measured evidence over claims,
one canonical implementation over duplicates,
and a safe deterministic fallback over an unvalidated learned component.
