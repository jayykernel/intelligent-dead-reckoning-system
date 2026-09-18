# Phase 8 Results: Predictive Outage Detection

## 1. Predictive Outage Detection Summary
The predictive outage detection module (`OutagePredictor`) uses trailing-window trend analysis of satellite counts and Carrier-to-Noise density ($C/N_0$) to emit a continuos `trust_score` ($T \in [0.0, 1.0]$). This score is used within `GNSSINSFusionEngine.step()` to dynamically scale the GNSS measurement noise covariance $R$ before a hard GNSS outage occurs.

## 2. Evaluation on Android GNSS Log
Validated using `pseudoranges_log_2016_08_22_14_45_50.txt`.

- **Predictive Capability**: The `OutagePredictor` successfully computes a `trust_score` based on signal quality trends.
- **Log Investigation**: The trace `pseudoranges_log_2016_08_22_14_45_50.txt` exhibits a dip in signal quality (min $C/N_0$ = 23.1 dB-Hz, min satellites = 9) rather than a complete signal blackout (tunnels/underpasses typicaly show $< 15\text{ dB-Hz}$ or $\le 3\text{ satellites}$). Predictive trend detection correctly reacts to the degradation in signal quality, and the fusion engine adaptively expands the GNSS measurement uncertainty covariance $R$ preemptively.
- **Next Steps**: A true tunnel/underpass trace is required for benchmarking predictive lead-time ahead of *complete signal loss*.

## 3. End-to-End Fusion Integration
- The dynamic trust signal (`trust_score`) is fully integrated into `GNSSINSFusionEngine`.
- GNSS measurement noise (SigmaPos, SigmaVel) is adaptively scaled based on trust:
  $$\sigma_{\text{pos, dynamic}} = \frac{\sigma_{\text{pos, nominal}}}{\max(0.1, \sqrt{\text{Trust}})}$$
- **Known Limitation**: As documented in `docs/PHASE6_RESULTS.md`, Phase 8 inherits the heading-observability gap during pure dead reckoning segments (GNSS outage). While Phase 8 improves fusion smoothness during transition, it does not solve the yaw-drift intrinsic to the sensor-fusion architecture documented in Phase 6.

## 4. Verification Plot
See `data/processed/phase8_eval/phase8_predictive_outage_detection.png` for the trend detection performance and trust-to-sigma coupling behavior on the current evaluation log.
