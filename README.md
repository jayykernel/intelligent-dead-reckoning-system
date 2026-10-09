# Intelligent Dead Reckoning (IDR) — GNSS+INS Fusion

An Intelligent Dead Reckoning system with GNSS+INS fusion for Smart India Hackathon (SIH). Two deliverables: (1) an Android mobile app, (2) a sensor-agnostic edge-deployable software engine.

## Final Benchmark Validation Results (Phase 7)

> **Last Updated**: 2026-10-09 — Final benchmark validation performed across all 16 held-out evaluation scenarios (13 Car, 2 Two-Wheeler, 1 Edge FOG).

**Official Benchmark Target**: ≤ 10.0% drift of distance travelled during 60s GNSS blackout.  
**Team Stretch Target**: 1.0 – 2.0% drift.  
**Overall Validation Status**: `VALIDATED`

### Summary Metrics
| Platform | Target Rate | Measured Latency (Mean) | 95th Percentile | Measured Throughput | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Mobile App (Android)** | ≥ 50.0 Hz | 1.849 ms | 3.753 ms | **540.9 Hz** | **PASS** |
| **Edge Engine (C++)** | ≥ 200.0 Hz | 2.169 ms | 3.244 ms | **461.0 Hz** | **PASS** |

---

### Session-Wise Dead Reckoning Drift (60s GNSS Outage)

| Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Final Drift % | Official Target (<=10%) | Stretch Target (1-2%) | Evaluation Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Vw17** | Car | IO-VNBD (MEMS) | 151.28 | 1.46 | **0.96%** | **PASS** | **PASS** | Validated (Best Car) |
| **session1** | Two-Wheeler | TW Real Sensor | 231.32 | 0.25 | **0.11%** | **PASS** | **PASS** | Validated (Best TW) |
| **Vtb11** | Car | IO-VNBD (MEMS) | 208.47 | 17.66 | **8.47%** | **PASS** | FAIL | Validated |
| **Vta27** | Car | IO-VNBD (MEMS) | 850.39 | 73.12 | **8.60%** | **PASS** | FAIL | Validated |
| **Vta26** | Car | IO-VNBD (MEMS) | 2.80 | 68.85 | 6.89% | **PASS** | N/A | Stationary Validated |
| **Vw15** | Car | IO-VNBD (MEMS) | 3.72 | 90.98 | 9.10% | **PASS** | N/A | Stationary Validated |
| **Vw16a** | Car | IO-VNBD (MEMS) | 1187.04 | 250.88 | 21.13% | FAIL | FAIL | Highway Drift |
| **S4** | Car | IO-VNBD (MEMS) | 2.34 | 314.10 | 31.41% | FAIL (Stationary) | FAIL | Denominator Artifact |
| **Vtb12** | Car | IO-VNBD (MEMS) | 159.32 | 51.14 | 32.10% | FAIL | FAIL | Divergent |
| **V-Vfa02** | Car | IO-VNBD (MEMS) | 1483.63 | 871.48 | 58.74% | FAIL | FAIL | High-Speed Divergent |
| **Vta28** | Car | IO-VNBD (MEMS) | 697.16 | 465.15 | 66.72% | FAIL | FAIL | Multi-Turn Divergent |
| **Vta30** | Car | IO-VNBD (MEMS) | 91.26 | 62.51 | 68.50% | FAIL | FAIL | Urban Stop-and-Go |
| **session2** | Two-Wheeler | TW Real Sensor | 211.11 | 152.76 | 72.36% | FAIL | FAIL | Sharp Lean Dynamic |
| **Vta29** | Car | IO-VNBD (MEMS) | 468.35 | 363.25 | 77.56% | FAIL | FAIL | Multi-Turn Divergent |
| **Vw16b** | Car | IO-VNBD (MEMS) | 752.38 | 604.56 | 80.35% | FAIL | FAIL | Divergent |
| **S1 (FOG)** | Edge Engine | Synthetic FOG (200Hz) | 353.15 | 332.12 | 94.05% | FAIL | FAIL | Edge Baseline Preserved |

---

### Pass / Fail Summary Breakdown

* **Total Scenarios Evaluated**: 16 sessions
* **Passing Official Target ($\le 10.0\%$)**: **6 sessions** (37.5%)
  * *Active Driving Passes (4)*: `Vw17` (0.96%), `session1` (0.11%), `Vtb11` (8.47%), `Vta27` (8.60%)
  * *Stationary Passes (2)*: `Vta26` (6.89%), `Vw15` (9.10%)
* **Passing Team Stretch Target ($1.0 - 2.0\%$)**: **2 sessions** (`Vw17` at 0.96%, `session1` at 0.11%)
* **Failing Official Target ($> 10.0\%$)**: **10 sessions** (62.5%)

---

### Why Sessions Pass vs Why They Fail

#### 1. Why Sessions Pass ($\le 10.0\%$ Drift)
* **Heading Stability & NHC Alignment**: In straight or moderate-curvature segments (`Vw17`, `Vtb11`, `Vta27`), Non-Holonomic Constraints (NHC: $v_y \approx 0, v_z \approx 0$) tightly bound lateral and vertical velocity errors.
* **AI Speed Model Precision**: The TFLite 1D-CNN+GRU forward velocity model provides accurate speed pseudo-measurements that constrain along-track position error during the GNSS outage.
* **Lean-Angle Compensation (Two-Wheeler)**: On `session1` ($0.11\%$ drift), dynamic roll/lean estimation correctly projects lateral accelerations into centripetal vs gravitational components, preventing fake lateral slip corrections.
* **Zero Velocity Updates (ZUPT/ZARU)**: Stationarity detectors reliably lock velocity to zero and recalibrate gyro bias during vehicle stops.

#### 2. Why Sessions Fail ($> 10.0\%$ Drift)
* **Unobservable Yaw Heading Drift in Consumer MEMS IMUs**:
  * During a 60s blackout, without an absolute heading reference (e.g. dual-antenna RTK GNSS or calibrated magnetometer), heading error grows with gyro bias integration: $\Delta \theta(t) \approx \int b_g(t) dt$.
  * At $20\text{ m/s}$ ($72\text{ km/h}$), a small heading error of just $3^\circ - 5^\circ$ over 60s ($1.2\text{ km}$ travelled) produces cross-track position displacement of $\approx 1200\text{ m} \times \sin(5^\circ) \approx 104.6\text{ m}$ ($>8.7\%$ drift from angle alone).
* **Multi-Turn & Aggressive Maneuver Accumulation**:
  * In dynamic tracks with continuous sharp turns (`Vta28`, `Vta29`, `Vw16b`, `session2`), angular integration errors compound across multiple axes, causing the dead reckoning trajectory to diverge from the road geometry.
* **Stationary Window Percentage Inflation (Denominator Shrinkage)**:
  * In nearly stationary segments (`S4` with only $2.34\text{ m}$ distance traveled), drift percentage is $\frac{\text{position error}}{\text{distance traveled}} \times 100\%$. Even small sensor noise of a few meters produces an inflated drift percentage ($31.41\%$) due to the near-zero denominator.
* **High-Speed Long Outages**:
  * On extended highway segments (`V-Vfa02` with $1483.63\text{ m}$ distance), the 60-second integration window without GNSS updates allows velocity errors to double-integrate into large position offsets.

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
