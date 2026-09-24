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
- **Best Case (Car)**: S4 at **0.66% drift** — meets both official *and* stretch targets. Confidence-scaled map-matching heading + position updates keep yaw drift under control throughout the 60s blackout.
- **Worst Case (Car)**: Vta26 at 51.97% drift — short outage distance (179 m) and aggressive turning geometry make this the hardest car trajectory.
- **S1 (Long Session)**: 20.47% drift — improved from 52.64% via NIS force-accept for GNSS measurements preventing pre-outage filter lockout, plus confidence-scaled map-matching. The 34-minute session with dense turns still accumulates heading error.
- **Best Case (Two-Wheeler)**: session1 at 21.98% drift — improved from 263.39% via phone-to-vehicle calibration gravity fix, AI speed filter disable, and confidence-scaled map-matching.
- **Worst Case (Two-Wheeler)**: session2 at 33.83% drift — improved from 58.46%.
- **Edge FOG Path**: S1 at 1440.06% drift — synthetic FOG data without absolute heading reference; gyroscope bias drift dominates.

### Improvements Since Baseline (Pre-Optimization):

| Session | Baseline Drift % | Current Drift % | Improvement |
| :--- | :--- | :--- | :--- |
| **S4** | 18.69% | **0.66%** | **96.5% reduction** |
| **S1** | 52.64% | **20.47%** | **61.1% reduction** |
| **Vta26** | 42.83% | **51.97%** | Regressed (outage window change) |
| **session1** | 263.39% | **21.98%** | **91.7% reduction** |
| **session2** | 58.46% | **33.83%** | **42.1% reduction** |

### Key Optimizations Applied:
1. **Confidence-Scaled Map-Matching Position Updates**: $\sigma_{\text{pos}} = \sigma_{\text{base}} / \sqrt{\text{confidence}}$, clamped to $[0.5\,\text{m},\, 10.0\,\text{m}]$. Tighter position during outage ($\sigma_{\text{base}}=1.0\,\text{m}$), relaxed when GNSS available ($\sigma_{\text{base}}=3.0\,\text{m}$).
2. **Confidence-Scaled Map-Matching Heading Updates**: $\sigma_{\text{heading}} = \sigma_{\text{base}} / \sqrt{\text{confidence}}$, clamped to $[0.5°,\, 20°]$. Base sigma = 1.5° during outage, 3.0° when GNSS aided.
3. **180° Bearing Ambiguity Resolution**: Road segment bearings from OSM graph edges are disambiguated against vehicle yaw — selects forward vs reverse bearing by minimum angular difference.
4. **AI Speed Filter Disabled for Two-Wheelers**: Car-trained speed model predicted ~19.57 m/s on two-wheelers (GT: ~4.72 m/s) — now bypassed for `vehicle_type == "two_wheeler"`.
5. **Upright-Driving Gravity Calibration**: Phone-to-vehicle calibrator detects >15° tilt difference between stationary and driving gravity vectors (common on two-wheelers with tilted phone mounts / side-stand parking) and uses upright driving gravity for alignment.
6. **NIS Force-Accept for GNSS Measurements**: `GNSS_POS`, `GNSS_VEL`, `GNSS_HEADING` added to force-accept list to prevent pre-outage filter lockout on long sessions with dynamic turns.
7. **ZUPT/ZARU at Stops**: When `is_stopped == True`, Zero Velocity Update (ZUPT) and Zero Angular Rate Update (ZARU) correct accumulated gyro bias drift.
8. **MAP_HEADING Innovation Wrapping**: Added `"MAP_HEADING"` to the angular innovation wrapping logic in `ekf.py` so map heading innovations are bounded to $[-\pi, \pi]$.

> **Remaining Limitation**: Sessions S1, Vta26, and both two-wheeler sessions still exceed the 10% official target. Primary cause is unobservable yaw heading drift in consumer MEMS IMUs during extended GNSS blackouts. Further improvements require: (1) evaluating all IO-VNBD sessions to identify trajectory-specific tuning opportunities, (2) active lean-angle NHC relaxation for two-wheelers, (3) tighter map-matching with higher-resolution road networks.

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
| **Outage Entry** (GNSS_AIDED $\to$ PURE_DEAD_RECKONING) | **0.1 s** (1 epoch, dwell-limited) | **3.1 s** (smooth 5x expansion) | $< 1.0 \text{ m} / 0.0002 \text{ m/s}$ | **PASS** |
| **Reacquisition (Short Outage)** | **0.1 s** (1 epoch, dwell-limited) | **1.2 s** (rapid contraction) | $< 1.0 \text{ m} / 0.02 \text{ m/s}$ | **PASS** |
| **Reacquisition (Long Outage)** | **0.1 s** | Intentional Rejection (NIS gating prevents state corruption) | Drift-correcting step | **PASS** |

---

## 4. NIS Innovation Gating Statistics

| Session | Category | GNSS Updates Evaluated | GNSS Accepted | GNSS Rejected | Acceptance Rate % | Gating Integrity |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **S4** | car | 5892 | 5861 | 31 | 99.5% | Passed (Rejects Divergent Fixes) |
| **S1** | car | 102286 | 101504 | 782 | 99.2% | Passed (Rejects Divergent Fixes) |
| **Vta26** | car | 2776 | 2675 | 101 | 96.4% | Passed (Rejects Divergent Fixes) |
| **session1** | two_wheeler | 5514 | 4832 | 682 | 87.6% | Passed (Rejects Divergent Fixes) |
| **session2** | two_wheeler | 2270 | 1600 | 670 | 70.5% | Passed (Rejects Divergent Fixes) |

---

## 5. Artifact Checklist
All evaluation plots and test logs are committed and inspectable:
- `eval/plots/benchmark_drift_summary.png`
- `eval/plots/benchmark_car_trajectories.png`
- `eval/plots/benchmark_tw_trajectories.png`
- `eval/plots/benchmark_edge_fog.png`
- `eval/plots/benchmark_mode_transitions.png`
