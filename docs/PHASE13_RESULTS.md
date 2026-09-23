# Full Benchmark Validation Report (Phase 13)

> **Updated 2026-09-23**: Re-run with TFLite properly installed and AI correction active. Previous numbers (S1: 78,711.28%, session1: 12,649.46%, session2: 5,135.61%) were invalid baselines produced while the TFLite runtime was missing — AI modules returned None and no speed correction was applied. The numbers below are the first honest benchmark with the full pipeline (including CNN+GRU SpeedFilter and covariance-scaled k=100 correction) running.

## 1. Dead Reckoning Drift Performance (60s GNSS Blackout)
**Official Benchmark Target**: $< 10.0\%$ of total distance travelled during GNSS outage.
**Team Stretch Target**: $1.0 - 2.0\%$ drift.

| Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Drift % | Official Target (<=10%) | Stretch Target (1-2%) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | Car | IO-VNBD (MEMS) | 498.70 | 285.24 | **57.20%** | FAIL | FAIL |
| **S1** | Car | IO-VNBD (MEMS) | 389.66 | 33853.72 | **8688.07%** | FAIL | FAIL |
| **Vta26** | Car | IO-VNBD (MEMS) | 179.73 | 165.98 | **92.35%** | FAIL | FAIL |
| **session1** | Two Wheeler | Bridge Synthetic | 231.32 | 1110.59 | **480.12%** | FAIL | FAIL |
| **session2** | Two Wheeler | Bridge Synthetic | 211.11 | 70.19 | **33.25%** | FAIL | FAIL |
| **S1** | Edge Fog | FOG Synthetic | 352.88 | 5482.40 | **1553.63%** | FAIL | FAIL |

### Summary of Drift Findings:
- **Best Case (Two-Wheeler)**: session2 at **33.25%** drift — closest to the 10% target.
- **Best Case (Car)**: S4 at **57.20%** drift.
- **Worst Case (Car)**: S1 at **8,688.07%** drift (down from 78,711.28% without AI — a **89% reduction** from AI correction alone).
- **Edge FOG Path**: S1 at **1,553.63%** drift (demonstrating significant improvement over MEMS path).

### AI Correction Impact (TFLite Active vs. Missing):
| Session | Without AI (stale) | With AI (current) | Reduction |
| :--- | :--- | :--- | :--- |
| S1 (Car) | 78,711.28% | 8,688.07% | **89.0%** |
| session1 (TW) | 12,649.46% | 480.12% | **96.2%** |
| session2 (TW) | 5,135.61% | 33.25% | **99.4%** |

> **Documented Limitation (Phase 6)**: No configuration meets the $\le 10\%$ target during extended 60s blackout. **Root cause**: unobservable yaw heading drift in consumer MEMS/FOG IMUs without absolute heading references. The magnetometer is disabled due to per-session calibration errors (1–140° offset, ±50–120° noise). AI speed correction bounds velocity magnitude effectively (89–99% drift reduction vs. baseline) but cannot correct heading — position error still diverges quadratically with heading error. See `docs/OPEN_QUESTIONS.md` and `docs/PHASE6_RESULTS.md` for full analysis.

---

## 2. GNSS+INS Fusion Update Rate & Wall-Clock Throughput

| Platform | Target Rate | Measured Latency (Mean) | 95th Percentile | Measured Throughput | Status | Hardware Note |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Mobile App (Android/Kotlin)** | 10.0 Hz | 1.412 ms | 2.486 ms | **708.2 Hz** | **PASS** | Evaluated on phone pipeline & emulator |
| **Edge Engine (C++/Python Wrapper)** | ~200.0 Hz | 1.048 ms | 1.624 ms | **954.0 Hz** | **PASS** | Measured on developer machine; not validated on edge hardware |

---

## 3. Seamless Mode Transition Latency

| Transition Scenario | State Machine Flag-Flip Latency | Covariance Settling Latency | State Vector Continuity (Delta p / v) | Benchmark Status |
| :--- | :--- | :--- | :--- | :--- |
| **Outage Entry** | **0.1 s** | **3.1 s** | $< 1.0 \text{ m} / 0.0002 \text{ m/s}$ | **PASS** |
| **Reacquisition (Short)** | **0.1 s** | **1.2 s** | $< 1.0 \text{ m} / 0.02 \text{ m/s}$ | **PASS** |

---

## 4. NIS Innovation Gating Statistics

| Session | Category | GNSS Updates Evaluated | GNSS Accepted | GNSS Rejected | Acceptance Rate % |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | car | 5892 | 3250 | 2642 | 55.2% |
| **S1** | car | 102286 | 2768 | 99518 | 2.7% |
| **Vta26** | car | 2776 | 2676 | 100 | 96.4% |
| **session1** | two_wheeler | 5514 | 23 | 5491 | 0.4% |
| **session2** | two_wheeler | 2270 | 1339 | 931 | 59.0% |

