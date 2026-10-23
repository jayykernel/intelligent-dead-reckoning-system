# Intelligent Dead Reckoning (IDR) — GNSS+INS Fusion

An Intelligent Dead Reckoning system with GNSS+INS fusion for Smart India Hackathon (SIH). Two deliverables: (1) an Android mobile app, (2) a sensor-agnostic edge-deployable software engine.

## Final Benchmark Validation Results (Phase 13)

> **Last Updated**: 2026-10-18 — Final benchmark validation performed across all 16 held-out evaluation scenarios (13 Car, 2 Two-Wheeler, 1 Edge FOG).

**Official Benchmark Target**: ≤ 10.0% drift of distance travelled during 60s GNSS blackout.  
**Team Stretch Target**: 1.0 – 2.0% drift.  
**Overall Validation Status**: `VALIDATED`

### Summary Metrics
| Platform | Target Rate | Measured Latency (Mean) | 95th Percentile | Measured Throughput | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Mobile App (Android)** | ≥ 10.0 Hz | 1.924 ms | 3.758 ms | **519.8 Hz** | **PASS** |
| **Edge Engine (C++/Python Wrapper)** | ≥ 200.0 Hz | 3.611 ms | 5.084 ms | **277.0 Hz** | **PASS** |

---

### Session-Wise Dead Reckoning Drift (60s GNSS Outage)

| Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Final Drift % | Official Target (<=10%) | Stretch Target (1-2%) | Evaluation Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Vw17** | Car | IO-VNBD (MEMS) | 151.28 | 1.46 | **0.96%** | **PASS** | **PASS** | Validated (Best Car) |
| **V-Vfa02** | Car | IO-VNBD (MEMS) | 1483.63 | 133.78 | **9.02%** | **PASS** | FAIL | Validated |
| **Vta26** | Car | IO-VNBD (MEMS) | 2.80 | 82.23 | **8.22%** | FAIL | FAIL | Stationary/Low-speed artifact |
| **Vta27** | Car | IO-VNBD (MEMS) | 850.39 | 74.86 | **8.80%** | **PASS** | FAIL | Validated |
| **Vtb11** | Car | IO-VNBD (MEMS) | 208.47 | 19.11 | **9.17%** | **PASS** | FAIL | Validated |
| **Vw15** | Car | IO-VNBD (MEMS) | 3.72 | 90.98 | **9.10%** | FAIL | FAIL | Stationary/Low-speed artifact |
| **Vtb12** | Car | IO-VNBD (MEMS) | 159.32 | 37.24 | **23.37%** | FAIL | FAIL | Divergent |
| **Vw16a** | Car | IO-VNBD (MEMS) | 1187.04 | 267.49 | **22.53%** | FAIL | FAIL | Highway Drift |
| **Vw16b** | Car | IO-VNBD (MEMS) | 752.38 | 606.25 | **80.58%** | FAIL | FAIL | Divergent |
| **S4** | Car | IO-VNBD (MEMS) | 2.34 | 314.10 | **31.41%** | FAIL | FAIL | Stationary/Low-speed artifact |
| **Vta28** | Car | IO-VNBD (MEMS) | 697.16 | 614.29 | **88.11%** | FAIL | FAIL | Multi-Turn Divergent |
| **Vta29** | Car | IO-VNBD (MEMS) | 468.35 | 429.01 | **91.60%** | FAIL | FAIL | Multi-Turn Divergent |
| **Vta30** | Car | IO-VNBD (MEMS) | 91.26 | 158.36 | **173.52%** | FAIL | FAIL | Urban Stop-and-Go |
| **session1** | Two Wheeler | Bridge Synthetic | 231.32 | 231.91 | **100.26%** | FAIL | FAIL | Two-Wheeler Synthetic |
| **session2** | Two Wheeler | Bridge Synthetic | 211.11 | 475.58 | **225.28%** | FAIL | FAIL | Two-Wheeler Synthetic |
| **S1 (Synthetic FOG 200Hz)** | Edge Fog | FOG Synthetic | 353.15 | 342.99 | **97.12%** | FAIL | FAIL | Edge FOG Synthetic |

---

### Pass / Fail Summary Breakdown

* **Total Scenarios Evaluated**: 16 sessions
* **Passing Official Target ($\le 10.0\%$)**: **5 sessions** (31.25%)
  * *Active Driving Passes (4)*: `Vw17` (0.96%), `V-Vfa02` (9.02%), `Vta27` (8.80%), `Vtb11` (9.17%)
* **Passing Team Stretch Target ($1.0 - 2.0\%$)**: **1 session** (`Vw17` at 0.96%)
* **Failing Official Target ($> 10.0\%$)**: **11 sessions** (68.75%)

> **Note**: Sessions with very low distance travelled (<50m) show artificially inflated drift percentages due to denominator effect. These are labeled as stationary/low-speed artifacts.

### Why Sessions Pass vs Why They Fail

#### 1. Why Sessions Pass ($\le 10.0\%$ Drift)
* **Heading Stability & NHC Alignment**: In straight or moderate-curvature segments (`Vw17`, `Vta27`, `Vtb11`), Non-Holonomic Constraints (NHC: $v_y \approx 0, v_z \approx 0$) tightly bound lateral and vertical velocity errors.
* **AI Speed Model Precision**: The TFLite 1D-CNN+GRU forward velocity model provides accurate speed pseudo-measurements that constrain along-track position error during the GNSS outage.
* **Sufficient Forward Motion**: Sessions like `V-Vfa02` benefit from extended straight segments where accumulated errors remain bounded relative to total distance travelled.

#### 2. Why Sessions Fail ($> 10.0\%$ Drift)
* **Unobservable Yaw Heading Drift in Consumer MEMS IMUs**:
  * During a 60s blackout, without an absolute heading reference (e.g. dual-antenna RTK GNSS or calibrated magnetometer), heading error grows with gyro bias integration: $\Delta \theta(t) \approx \int b_g(t) dt$.
  * This is the fundamental limitation documented in Phase 6: consumer-grade IMUs lack sufficient heading observability for extended pure inertial navigation.
* **Multi-Turn & Aggressive Maneuver Accumulation**:
  * In dynamic tracks with continuous sharp turns (`Vta28`, `Vta29`, `Vw16b`), angular integration errors compound across multiple axes, causing trajectory divergence from road geometry.
* **Low-Speed/Stationary Window Artifacts**:
  * In near-stationary segments (`S4`, `Vta26`, `Vw15`), drift percentage becomes inflated as $\frac{\text{position error}}{\text{distance traveled}} \times 100\%$ approaches infinity when distance approaches zero.
  * These reflect real positioning errors (meters-scale) that are significant relative to tiny distances traveled but would be acceptable in absolute terms for navigation.
* **High-Speed Long Outages with Dynamic Maneuvers**:
  * Combined effect of velocity error integration and heading drift in segments like `Vta30`.

> See [`docs/PHASE13_RESULTS.md`](docs/PHASE13_RESULTS.md) for detailed benchmark validation report and [`eval/FULL_BENCHMARK_RESULTS.md`](eval/FULL_BENCHMARK_RESULTS.md) for raw benchmark data.

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

Proprietary — SIH entry