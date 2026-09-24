# Intelligent Dead Reckoning (IDR) — GNSS+INS Fusion

An Intelligent Dead Reckoning system with GNSS+INS fusion for Smart India Hackathon (SIH). Two deliverables: (1) an Android mobile app, (2) a sensor-agnostic edge-deployable software engine.

## Benchmark Results (Phase 13)

> **Last Updated**: 2026-09-24 — Post confidence-scaled map-matching optimizations.

**Official Target**: ≤ 10% drift during 60s GNSS blackout. **Stretch Target**: 1–2% drift.

| Session | Vehicle | Outage Dist (m) | Final Error (m) | Drift % | Official Target |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | Car | 498.70 | 3.27 | **0.66%** | ✅ PASS |
| **S1** | Car | 389.66 | 79.76 | **20.47%** | ❌ FAIL |
| **Vta26** | Car | 179.73 | 93.41 | **51.97%** | ❌ FAIL |
| **session1** | Two-Wheeler | 231.32 | 50.85 | **21.98%** | ❌ FAIL |
| **session2** | Two-Wheeler | 211.11 | 71.42 | **33.83%** | ❌ FAIL |
| **S1 FOG 200Hz** | Edge (Synthetic) | 352.88 | 5081.63 | **1440.06%** | ❌ FAIL |

> **S4 meets both the official ≤10% and the stretch 1–2% targets.** Remaining sessions are actively being optimised. See [`eval/FULL_BENCHMARK_RESULTS.md`](eval/FULL_BENCHMARK_RESULTS.md) for the full report including NIS gating statistics, throughput benchmarks, and detailed optimisation notes.

---

## Project Structure

```
/docs                     Architecture, phase checklist, novelty spec, benchmarks
/data
  /raw                    IO-VNBD + any collected data, untouched
  /processed              Cleaned/segmented data used for training
/training                 Offline model training code (cloud/desktop side)
  /models                 Saved trained models (checkpoints, .tflite exports)
/engine                   Core sensor-agnostic fusion engine (shared library)
  /calibration            Alignment & calibration engine
  /ai_filters             AI Speed & Vibration Filter
  /nhc_zupt               NHC + ZUPT (incl. lean-compensated)
  /fusion                 GNSS+INS Fusion Engine (EKF/UKF + AI correction)
  /map_matching           Map-Matching Filter
  /outage_prediction      Predictive GNSS Outage Detection
/mobile                   Android app (consumes /engine via TFLite + shared logic)
/edge                     Edge-deployable engine packaging (consumes /engine)
/eval                     Benchmark scripts, drift measurement, plots
/screening_package        Screening-round deliverable (model + plots + writeup)
```

## Quick Start

### Python Environment (Training & Engine)

```bash
cd training
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### Android App (Mobile)

```bash
cd mobile
./gradlew assembleDebug
```

### Edge Engine

```bash
cd edge
# Build instructions to be added in Phase 12
```

### Run Full Benchmark

```bash
training\venv\Scripts\python.exe eval/run_full_benchmark.py
```

## Documentation

- [Architecture](docs/ARCHITECTURE.md) — System design, module contracts, data flow
- [Novelty Specification](docs/NOVELTY_SPEC.md) — Locked feature specifications (N1–N8)
- [Phase Checklist](docs/PHASE_CHECKLIST.md) — Execution order and exit criteria
- [Benchmarks](docs/BENCHMARKS.md) — Official targets and stretch goals
- [Phase 13 Results](docs/PHASE13_RESULTS.md) — Full benchmark validation report
- [Full Benchmark Results](eval/FULL_BENCHMARK_RESULTS.md) — Detailed per-session results with NIS statistics
- [Open Questions](docs/OPEN_QUESTIONS.md) — Decisions log

## Development Protocol

See [CLAUDE.md](CLAUDE.md) for the project constitution: non-negotiable rules, execution protocol, git commit convention, and repository structure.

## License

Proprietary — SIH entry.
