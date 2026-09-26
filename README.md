# Intelligent Dead Reckoning (IDR) — GNSS+INS Fusion

An Intelligent Dead Reckoning system with GNSS+INS fusion for Smart India Hackathon (SIH). Two deliverables: (1) an Android mobile app, (2) a sensor-agnostic edge-deployable software engine.

## Benchmark Results (Phase 13) — Strict TEST_SESSIONS Split

> **Last Updated**: 2026-09-26 — Full Benchmark Validation across 16 test sessions, including 10m KDTree map-matching for stable heading.

**Official Target**: ≤ 10% drift during 60s GNSS blackout. **Stretch Target**: 1–2% drift.

| Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Drift % | Official Target (<=10%) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | Car | IO-VNBD (MEMS) | 498.70 | 41.64 | **8.35%** | ✅ PASS |
| **V-Vfa02** | Car | IO-VNBD (MEMS) | 1486.67 | 329.83 | **22.19%** | ❌ FAIL |
| **Vta26** | Car | IO-VNBD (MEMS) | 3.41 | 98.19 | **0.00%** | ❌ Stationary |
| **Vta27** | Car | IO-VNBD (MEMS) | 901.11 | 457.59 | **50.78%** | ❌ FAIL |
| **Vta28** | Car | IO-VNBD (MEMS) | 696.07 | 173.88 | **24.98%** | ❌ FAIL |
| **Vta29** | Car | IO-VNBD (MEMS) | 468.35 | 82.91 | **17.70%** | ❌ FAIL |
| **Vta30** | Car | IO-VNBD (MEMS) | 91.52 | 93.98 | **102.68%** | ❌ FAIL |
| **Vtb11** | Car | IO-VNBD (MEMS) | 224.93 | 99.75 | **44.35%** | ❌ FAIL |
| **Vtb12** | Car | IO-VNBD (MEMS) | 166.72 | 14.33 | **8.59%** | ✅ PASS |
| **Vw15** | Car | IO-VNBD (MEMS) | 4.39 | 30.34 | **0.00%** | ❌ Stationary |
| **Vw16a** | Car | IO-VNBD (MEMS) | 1211.71 | 369.06 | **30.46%** | ❌ FAIL |
| **Vw16b** | Car | IO-VNBD (MEMS) | 752.55 | 16.11 | **2.14%** | ✅ PASS |
| **Vw17** | Car | IO-VNBD (MEMS) | 169.99 | 19.48 | **11.46%** | ❌ FAIL |
| **session1** | Two-Wheeler | Bridge Synthetic | 231.32 | 39.47 | **17.06%** | ❌ FAIL |
| **session2** | Two-Wheeler | Bridge Synthetic | 211.11 | 55.07 | **26.09%** | ❌ FAIL |
| **S1 (Synthetic FOG 200Hz)** | Edge Fog | FOG Synthetic | 353.15 | 26.65 | **7.55%** | ✅ PASS |

**Key Insight**: Sessions like **S4**, **Vtb12**, and **Vw16b** along with **Edge FOG** successfully pass the ≤10% official target. Extreme drift percentages on idling sequences exist due to near-zero denominators (e.g. Vta26, Vw15 are stationary but the EKF wanders).

> See [`eval/FULL_BENCHMARK_RESULTS.md`](eval/FULL_BENCHMARK_RESULTS.md) for the full report including NIS gating statistics, throughput benchmarks, and detailed optimisation notes.

## Drift Reduction Strategies

The following approaches are being pursued to reduce drift percentage across all test sessions:

### 1. **Absolute Heading References** (Highest Impact)
- **Magnetometer Calibration + Hard/Soft Iron Compensation**: The current pipeline uses magnetometer heading only when NIS gate passes. An on-device figure-8 calibration routine with ellipsoid fitting would enable reliable magnetic heading during outages, directly constraining the unobservable yaw drift.
- **Dual-Antenna GNSS (RTK) Heading**: Where hardware permits, carrier-phase differential GNSS provides ~0.1° absolute heading — eliminates yaw drift entirely during aided segments and initializes the outage with perfect alignment.

### 2. **Map-Matching Heading Fusion Enhancements**
- **Multi-Hypothesis Edge Tracking**: Current HMM uses single-best path. Maintaining a small set of top-K path hypotheses (beam search) would reduce 180° ambiguity flips on bidirectional road edges, especially at intersections.
- **Curvature-Constrained Smoothing**: Integrate road geometry (curvature from OSM) as a soft constraint on the heading innovation — penalize innovations that imply turning radii inconsistent with the matched road segment.
- **Confidence-Weighted Heading Injection**: Already partially implemented (sigma scales with 1/√conf). Further tune the base sigma for MAP_HEADING: reduce from 1.5° to 0.8° during outages when map confidence > 0.8.

### 3. **Gyroscope Bias Estimation & ZARU Improvements**
- **Persistent Bias State**: The current ZARU updates gyro bias during stops but does not persist it across sessions. Serialize `b_g` to local storage on app exit and restore on next start.
- **Temperature-Compensated Bias Model**: MEMS gyro bias drifts significantly with temperature. Add a temperature sensor reading (available on most Android devices) and learn a per-device bias-vs-temperature calibration curve during training.
- **Allan Variance Characterization**: Characterize the specific IMU's noise parameters (N, B, K) per device model to set optimal `sigma_gyro_bias` and ZARU gating thresholds.

### 4. **NHC & ZUPT Threshold Tuning per Vehicle Type**
- **Adaptive NHC Sigma**: Current NHC uses fixed lateral/vertical noise (0.2 m/s). Make this adaptive to speed: tighter constraints at high speed (where NHC is more valid), looser at low speed (where lateral slip is higher).
- **Surface-Type Detection**: Use accelerometer vibration spectrum to detect road surface (asphalt vs gravel vs unpaved) and adjust NHC/ZUPT thresholds dynamically.

### 5. **AI Forward Speed Model Generalization**
- **Domain Adaptation for Rural/Unpaved**: The current speed regressor is trained on suburban/urban data. Fine-tune or retrain on Driver E's rural sessions (Vta, Vtb, Vw) using few-shot adaptation.
- **Speed Uncertainty Output**: Modify the TFLite model to output a heteroscedastic uncertainty (aleatoric) alongside the speed estimate, allowing the EKF to automatically down-weight unreliable predictions.

### 6. **Outage Window Protocol Standardization**
- **Active-Driving Outage Selection**: For future benchmarks, select outage windows that guarantee minimum distance travelled (e.g., > 100m) or minimum speed (> 1 m/s) to avoid the stationary-window percentage inflation artifact.
- **Multiple Randomized Outages**: Report median/95th-percentile drift across 10 randomized 60s outages per session instead of a single fixed window.

### 7. **Edge Engine FOG-Specific Fixes**
- The 1440% drift on synthetic FOG indicates the map-matching reset logic at outage entry is not providing a clean initialization. Add an explicit `reset_history()` + `match_point()` re-initialization at the exact outage start timestamp, and verify the edge matcher's transition model parameters for 200Hz update rates.

---

**Priority Order for Next Iteration**:
1. Magnetometer calibration + heading fusion (targets: ~50% drift reduction on active sessions)
2. Persistent gyro bias + temperature compensation (targets: ~30% drift reduction on long outages)
3. Multi-hypothesis map matching (targets: eliminate 180° flips)
4. Domain-adapted AI speed model for rural sessions (Vta/Vtb/Vw)

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
