# Intelligent Dead Reckoning (IDR) — GNSS+INS Fusion

An Intelligent Dead Reckoning system with GNSS+INS fusion for Smart India Hackathon (SIH). Two deliverables: (1) an Android mobile app, (2) a sensor-agnostic edge-deployable software engine.

## Final Benchmark Validation Results (Phase 7)

> **Last Updated**: 2026-10-09 — Final benchmark validation performed across 16 held-out scenarios.

**Official Target**: ≤ 10.0% drift during 60s GNSS blackout.
**Status**: `VALIDATED`

### Summary Metrics
| Platform | Target Rate | Measured Latency (Mean) | 95th Percentile | Measured Throughput | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Mobile App (Android)** | ≥ 50.0 Hz | 1.849 ms | 3.753 ms | **540.9 Hz** | **PASS** |
| **Edge Engine (C++)** | ≥ 200.0 Hz | 2.169 ms | 3.244 ms | **461.0 Hz** | **PASS** |

### Drift Performance Snapshot
Multiple dynamic car scenarios (`Vta27` 8.60%, `Vtb11` 8.47%, `Vw17` 0.96%) and two-wheeler sessions (`session1` 0.11%) successfully pass the official ≤ 10.0% drift target.

> See [`docs/FINAL_COMPARISON.md`](docs/FINAL_COMPARISON.md) for detailed baseline-to-final drift comparisons and [`eval/final_results_phase7.json`](eval/final_results_phase7.json) for raw benchmark data.

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

### Run Final Benchmark

```bash
training\venv\Scripts\python.exe eval/run_full_benchmark.py
```

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Novelty Specification](docs/NOVELTY_SPEC.md)
- [Phase Checklist](docs/PHASE_CHECKLIST.md)
- [Final Comparison](docs/FINAL_COMPARISON.md) — Final Phase 7 validation report
- [Implementation Progress](docs/IMPLEMENTATION_PROGRESS.md)
- [Open Questions](docs/OPEN_QUESTIONS.md)

## Development Protocol

See [CLAUDE.md](CLAUDE.md) for the project constitution: non-negotiable rules, execution protocol, git commit convention, and repository structure.

## License

Proprietary — SIH entry.
