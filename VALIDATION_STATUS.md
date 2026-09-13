# Verification and Validation Evidence Matrix
**Project:** SIH PS 26168 — AI/ML based Intelligent Dead Reckoning System

This matrix classifies the exact state of project verification and validation. Every phase must explicitly classify its evidence.

## Evidence Classifications
- **IMPLEMENTED**: Code exists and compiles, no guarantee of correctness.
- **UNIT TESTED**: Component-level automated tests written and passing.
- **INTEGRATION TESTED**: Sub-systems verified running together.
- **SYNTHETICALLY VALIDATED**: Evaluated deterministically using generated proxy data.
- **REAL-DATA VALIDATED**: Tested against real-world physical IMU/GNSS sensor datasets from target environments.
- **NOT VALIDATED**: Needs evidence.

---

## Phase Evidence Matrix

| Phase | Capability | Implemented | Unit Tested | Integration Tested | Synthetically Validated | Real-Data Validated |
|---|---|:---:|:---:|:---:|:---:|:---:|
| **1** | Project Foundation | ✅ | ✅ | N/A | N/A | N/A |
| **2** | Sensor Abstraction & Sync | ✅ | ✅ | ✅ | ✅ | ❌ |
| **3** | Sensor Calibration | ✅ | ✅ | ✅ | ✅ | ❌ |
| **4** | Phone-to-Vehicle Alignment | ✅ | ✅ | ✅ | ✅ | ❌ |
| **5** | INS Mechanization Baseline | ✅ | ✅ | ✅ | ✅ | ❌ |
| **6** | Error-State Kalman Filter (ESKF)| ✅ | ✅ | ✅ | ✅ | ❌ |
| **7** | ZUPT & ML Velocity (Base) | ✅ | ✅ | ✅ | ✅ | ❌ |
| **8** | GNSS Integrity | ✅ | ✅ | ✅ | ✅ | ❌ |
| **9** | Vibration Analysis | ✅ | ✅ | ✅ | ✅ | ❌ |
| **10** | ML Model Training & Deployment| ✅ | ✅ | ✅ | ✅ | ❌ |
| **11** | Real-Data Generalization | ❌ | ❌ | ❌ | ❌ | ❌ BLOCKED |
| **12+**| Upcoming Roadmap Features | ❌ | ❌ | ❌ | ❌ | ❌ |

### Phase 1–6: Frozen Golden Foundation
Phases 1 through 6 encompass the core deterministic pipeline tracking pure inertial navigation, coordinate frame transformations, error-state covariance bounds, and offline/online calibration mechanisms. These are marked as synthetically validated via rigorous numerical test models confirming algebraic drift relationships. Real-world physical performance profiles will be integrated during Phase 11.

### Phase 10: ML Velocity Aiding Limitation Notice
The Temporal Convolutional Network (TCN) implementation generates accurate velocity constraints under synthetic distribution assumptions. However, **Universal ML Validation cannot be claimed**: integration of these predictions degraded initial navigation drift bounds, requiring an autonomous rate-limiting policy (throttling update rates) to restore safe ESKF performance parameters. The system prevents ML constraints from directly overwriting the core deterministic navigation states. **Real-world physical validations remain completely unverified.**

### Phase 11 Blockage
As of the current project state, all forms of real-world datasets representing operational vehicle environments (`data/raw/*`, `data/processed/*`) are unavailable. Phase 11 remains structurally blocked until experimental logging operations fetch genuine field dynamics.
