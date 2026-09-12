# Dead Reckoning Navigation System

**SIH PS 26168** — AI/ML based Intelligent Dead Reckoning System for Seamless Navigation

## Overview

A physics-informed, machine-learning-augmented inertial navigation system designed for seamless GNSS-denied vehicle navigation on smartphones (~10 Hz) and edge devices (~200 Hz).

**Core Principle:** ML estimates useful quantities (velocity, motion state, reliability). Physics maintains consistency. The system always quantifies its own uncertainty.

## Architecture

```
Sensors → Calibration → Alignment → [ML Intelligence + INS Mechanization]
    → Error-State Kalman Filter → Adaptive Constraints → Map Matching
    → Navigation Integrity → Output (Position + Uncertainty + Confidence)
```

See `docs/ARCHITECTURE.md` for the full mathematical specification.

## Project Status

| Phase | Description | Status |
|---|---|---|
| Phase 0 | Project Audit & Documentation | ✅ Complete |
| Phase 1 | Project Infrastructure | ✅ Complete |
| Phase 2–7 | Core Navigation (Sensors → EKF) | ✅ Complete |
| Phase 8–13 | Intelligence Layer (GNSS, Vibration, ML, Vehicles) | 🔲 Not Started |
| Phase 14–16 | Map & Integrity | 🔲 Not Started |
| Phase 17–19 | Evaluation & Benchmarks | 🔲 Not Started |
| Phase 20–22 | Optimization & Demo | 🔲 Not Started |

## Documentation

| Document | Description |
|---|---|
| [PROJECT_AUDIT.md](docs/PROJECT_AUDIT.md) | Repository audit, risks, and status assessment |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | Full system architecture with mathematical foundations |
| [IMPLEMENTATION_ROADMAP.md](docs/IMPLEMENTATION_ROADMAP.md) | Detailed phase milestones and deliverables |
| [REQUIREMENTS_TRACEABILITY.md](docs/REQUIREMENTS_TRACEABILITY.md) | PS 26168 requirements mapped to modules and tests |
| [NOVELTY_MATRIX.md](docs/NOVELTY_MATRIX.md) | Honest novelty assessment with evidence requirements |
| [FAILURE_ANALYSIS.md](docs/FAILURE_ANALYSIS.md) | Anticipated and observed failure modes |

## Development Rules

1. **No phase starts before the previous phase passes tests.**
2. **No novelty claims without experimental evidence.**
3. **No fabricated results.** If performance is poor, document it.
4. **Uncertainty is always reported.** The system says "I don't know" rather than lying.
5. **ML augments physics.** ML never replaces the navigation filter.

## License

_To be determined._
