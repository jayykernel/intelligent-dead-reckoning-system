# Full Benchmark Validation Report (Phase 13)

> **Last Updated**: 2026-09-24 — Post confidence-scaled map-matching + calibrator + AI speed filter optimizations.

## 1. Dead Reckoning Drift Performance (60s GNSS Blackout)
**Official Benchmark Target**: ${{ < 10.0\% }}$ of total distance travelled during GNSS outage.
**Team Stretch Target**: ${{ 1.0 - 2.0\% }}$ drift.

| Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Drift % | Official Target (<=10%) | Stretch Target (1-2%) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | Car | IO-VNBD (MEMS) | 498.70 | 3.27 | **0.66%** | **PASS ✅** | **PASS ✅** |
| **S1** | Car | IO-VNBD (MEMS) | 389.66 | 79.76 | **20.47%** | FAIL | FAIL |
| **Vta26** | Car | IO-VNBD (MEMS) | 179.73 | 93.41 | **51.97%** | FAIL | FAIL |
| **session1** | Two Wheeler | Bridge Synthetic | 231.32 | 50.85 | **21.98%** | FAIL | FAIL |
| **session2** | Two Wheeler | Bridge Synthetic | 211.11 | 71.42 | **33.83%** | FAIL | FAIL |
| **S1 (Synthetic FOG 200Hz)** | Edge Fog | FOG Synthetic | 352.88 | 5081.63 | **1440.06%** | FAIL | FAIL |

### Summary of Drift Findings:
- **Best Case (Car)**: S4 at **0.66% drift** — meets both official *and* stretch targets.
- **Worst Case (Car)**: Vta26 at 51.97% drift — short outage distance (179 m) and aggressive turning geometry.
- **S1 (Long Session)**: 20.47% drift — improved from 52.64% via NIS force-accept and confidence-scaled map-matching.
- **Best Case (Two-Wheeler)**: session1 at 21.98% drift — improved from 263.39%.
- **Worst Case (Two-Wheeler)**: session2 at 33.83% drift — improved from 58.46%.
- **Edge FOG Path**: S1 at 1440.06% drift — synthetic FOG without absolute heading reference.

### Improvements Since Baseline (Pre-Optimization):

| Session | Baseline Drift % | Current Drift % | Improvement |
| :--- | :--- | :--- | :--- |
| **S4** | 18.69% | **0.66%** | **96.5% reduction** |
| **S1** | 52.64% | **20.47%** | **61.1% reduction** |
| **session1** | 263.39% | **21.98%** | **91.7% reduction** |
| **session2** | 58.46% | **33.83%** | **42.1% reduction** |

### Key Optimizations Applied:
1. **Confidence-Scaled Map-Matching Position Updates**: $\sigma_{\text{pos}} = \sigma_{\text{base}} / \sqrt{\text{confidence}}$, clamped to $[0.5\,\text{m},\, 10.0\,\text{m}]$.
2. **Confidence-Scaled Map-Matching Heading Updates**: $\sigma_{\text{heading}} = \sigma_{\text{base}} / \sqrt{\text{confidence}}$, clamped to $[0.5°,\, 20°]$.
3. **180° Bearing Ambiguity Resolution**: Disambiguates forward vs reverse road bearing against vehicle yaw.
4. **AI Speed Filter Disabled for Two-Wheelers**: Car-trained model produced ~19.57 m/s predictions on two-wheelers (GT: ~4.72 m/s).
5. **Upright-Driving Gravity Calibration**: Detects >15° tilt from side-stand parking and uses upright driving gravity.
6. **NIS Force-Accept for GNSS**: Prevents pre-outage filter lockout on long sessions.
7. **ZUPT/ZARU at Stops**: Corrects accumulated gyro bias when vehicle is stationary.
8. **MAP_HEADING Innovation Wrapping**: Bounded to $[-\pi, \pi]$ in EKF update.

> **Remaining Limitation**: Sessions S1, Vta26, and both two-wheeler sessions still exceed the 10% official target. Primary cause is unobservable yaw heading drift in consumer MEMS IMUs during extended GNSS blackouts without absolute heading references.

---

## 2. GNSS+INS Fusion Update Rate & Wall-Clock Throughput

| Platform | Target Rate | Measured Latency (Mean) | 95th Percentile | Measured Throughput | Status | Hardware Note |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Mobile App (Android/Kotlin)** | 10.0 Hz | 1.618 ms | 2.864 ms | **618.1 Hz** | **PASS** | Evaluated on phone pipeline & emulator |
| **Edge Engine (C++/Python Wrapper)** | ~200.0 Hz | 1.267 ms | 1.827 ms | **789.0 Hz** | **PASS** | Measured on developer machine CPU; not yet validated on target embedded edge hardware. |

---

## 3. Seamless Mode Transition Latency

| Transition Scenario | State Machine Flag-Flip Latency | Covariance Settling Latency | State Vector Continuity (Delta p / v) | Benchmark Status |
| :--- | :--- | :--- | :--- | :--- |
| **Outage Entry** (GNSS_AIDED → PURE_DEAD_RECKONING) | **0.1 s** (1 epoch, dwell-limited) | **3.1 s** (smooth 5x expansion) | < 1.0 m / 0.0002 m/s | **PASS** |
| **Reacquisition (Short Outage)** | **0.1 s** (1 epoch, dwell-limited) | **1.2 s** (rapid contraction) | < 1.0 m / 0.02 m/s | **PASS** |
| **Reacquisition (Long Outage)** | **0.1 s** | Intentional Rejection (NIS gating prevents state corruption) | Drift-correcting step | **PASS** |

---

## 4. NIS Innovation Gating Statistics

| Session | Category | GNSS Updates Evaluated | GNSS Accepted | GNSS Rejected | Acceptance Rate % | Gating Integrity |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | car | 5892 | 5861 | 31 | 99.5% | Passed |
| **S1** | car | 102286 | 101504 | 782 | 99.2% | Passed |
| **Vta26** | car | 2776 | 2675 | 101 | 96.4% | Passed |
| **session1** | two_wheeler | 5514 | 4832 | 682 | 87.6% | Passed |
| **session2** | two_wheeler | 2270 | 1600 | 670 | 70.5% | Passed |

---

## 5. Artifact Checklist
All evaluation plots and test logs are committed and inspectable:
- `eval/plots/benchmark_drift_summary.png`
- `eval/plots/benchmark_car_trajectories.png`
- `eval/plots/benchmark_tw_trajectories.png`
- `eval/plots/benchmark_edge_fog.png`
- `eval/plots/benchmark_mode_transitions.png`
