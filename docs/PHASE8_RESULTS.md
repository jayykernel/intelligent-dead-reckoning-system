# Phase 8 Results: Predictive Outage Detection

## 1. Primary Validation: Real Outage & Signal Blackout (GSDC Urban Canyon Drive)

- **Dataset**: Google Smartphone Decimeter Challenge (GSDC) — `2021-04-28-US-SJC-1` (Downtown San Jose, Google Pixel 4, `data/raw/gsdc_sjc/2021-04-28-US-SJC-1_Pixel4_GnssLog.txt`).
- **Signal Conditions**: Real urban canyon drive with dense high-rises and overhead structural blockages, containing genuine multi-second GNSS blackout events where tracked satellite counts collapse below the minimum 4-satellite threshold required for a 3D GNSS fix.
- **Total Tracked Epochs**: 2019 epochs (1 Hz raw Android `GnssLogger` stream).
- **Satellite Range**: 2 to 21 satellites (nominal mean: 18.0 satellites).
- **$C/N_0$ Range**: 20.4 to 35.4 dB-Hz (nominal mean: 32.0 dB-Hz).

### Measured Predictive Lead-Time Ahead of Real Blackouts

1. **Outage Event 1 (Overpass / Deep Canyon Blockage)**:
   - **Nominal Tracking ($t \le 71.0\text{s}$)**: 18–19 satellites, mean $C/N_0 = 33.7\text{ dB-Hz}$, Trust = 0.96.
   - **Early Trust Shift Onset ($t = 72.0\text{s}$)**: Tracked satellites drop to 14, $C/N_0$ drops to $29.8\text{ dB-Hz}$, trend slope triggers early warning: **Trust drops to 0.70**.
   - **Severe Degradation ($t = 73.0\text{s}$)**: Tracked satellites collapse to 5, $C/N_0 = 23.5\text{ dB-Hz}$, **Trust collapses to 0.00**.
   - **Hard Blackout Onset ($t = 74.0\text{s}$)**: Tracked satellites collapse to **3 satellites** (complete loss of GNSS fix).
   - **Actual Predictive Lead-Time**: **2.0 seconds** ahead of complete fix loss.

2. **Outage Event 2 (Overhead Structural Blackout)**:
   - **Nominal Tracking ($t \le 1958.0\text{s}$)**: 16–17 satellites, mean $C/N_0 = 31.9\text{ dB-Hz}$, Trust = 0.86–0.91.
   - **Early Trust Shift Onset ($t = 1959.0\text{s}$)**: Tracked satellites collapse by 50% to 8 satellites, $C/N_0 = 27.8\text{ dB-Hz}$, trend slope triggers early warning: **Trust drops to 0.27**.
   - **Deep Degradation ($t = 1960.0\text{s}$)**: Tracked satellites drop to 12 (with degraded carrier power $C/N_0 = 25.7\text{ dB-Hz}$), **Trust = 0.38**.
   - **Hard Blackout Onset ($t = 1961.0\text{s}$ to $1963.0\text{s}$)**: Tracked satellites collapse to **2, 2, and 3 satellites** across a 3-second continuous outage window (complete loss of GNSS fix).
   - **Actual Predictive Lead-Time**: **2.0 seconds** ahead of complete fix loss.

### Practical Significance & Sample Size Context
- **Practical Significance of 2.0s Lead Time**:
  - **Distance Traveled**: At typical urban driving speeds of 40–50 km/h (11.1–13.8 m/s), a 2.0-second warning covers **22 to 28 meters** of road travel. This corresponds directly to the physical entrance/shadow zone preceding a tunnel portal or multi-lane overpass.
  - **Fusion Smoothing**: Because the mobile EKF updates at 10 Hz, 2.0 seconds provides **20 discrete filter prediction/update cycles**. This is more than sufficient time to ramp the GNSS measurement covariance smoothly from $R = (5.0\text{ m})^2 \to (50.0\text{ m})^2$, effectively decoupling the filter state from degraded GNSS measurements before the hard fix drop occurs, eliminating state jumps.
- **Sample Size Context**:
  - An exhaustive scan across the entire 2019-epoch (33.6-minute) dataset identified **exactly two discrete blackout events** (where tracked satellites fell to $\le 3$). 
  - The measured 2.0-second lead time across both events is preliminary and should be viewed as an indicative figure for this detection window configuration ($\text{window} = 4.0\text{s}$ at $1\text{ Hz}$), rather than an exhaustive multi-session benchmark.

---

## 2. End-to-End Fusion Integration & Adaptive $R$ Scaling

The `OutagePredictor` is wired end-to-end into `GNSSINSFusionEngine.step()`, consuming real-time raw GNSS signals (`avg_cn0`, `sat_count`, `accuracy_m`).

- **Dynamic Measurement Uncertainty**:
  $$\sigma_{\text{pos, dynamic}} = \frac{\sigma_{\text{pos, nominal}}}{\max(0.1, \sqrt{\text{Trust}})}$$
- **Nominal State ($\text{Trust} = 1.0$)**: $\sigma_{\text{pos}} = 5.0\text{ m}$.
- **Pre-Outage De-weighted State ($\text{Trust} \to 0.0$)**: $\sigma_{\text{pos}}$ expands dynamically up to **$50.0\text{ m}$** (a $100\times$ increase in measurement covariance $R$).
- **Effect**: By de-weighting GNSS fixes 2.0 seconds before complete blackout, the Kalman filter avoids injecting noisy multi-path/attenuated measurements into the state vector immediately prior to outage, ensuring a smooth transition into pure INS dead reckoning.

### Scope & Architectural Note (Phase 6 Limitation Inheritance)
- **Inherited Limitation**: While Phase 8 successfully solves the pre-outage detection and adaptive de-weighting problem (preventing filter shock during entry into GNSS denial), it **inherits Phase 6's documented heading-observability limitation** during the outage itself. Consumer smartphone IMUs continue to drift in yaw open-loop when GNSS course-over-ground heading is unavailable.

---

## 3. Supplementary Context: Routine Signal Degradation Test

- **Dataset**: Google GnssLogger baseline trace `pseudoranges_log_2016_08_22_14_45_50.txt`.
- **Nature of Event**: Signal quality variation without blackout (satellite count minimum = 9, $C/N_0$ minimum = 23.1 dB-Hz).
- **Result**: `OutagePredictor` detected the carrier-to-noise dip and modulated trust proportionally ($1.0 \to 0.22$), confirming that the trailing-window trend detector remains responsive across the entire dynamic range of signal quality.

---

## 4. Verification Artifacts

- **Multi-panel verification plot**: `data/processed/phase8_eval/phase8_predictive_outage_detection.png` (displays raw satellite counts, $C/N_0$, predictive trust signal, and dynamic $\sigma_{\text{pos}}$ side-by-side for both blackout events).
- **Evaluation script**: `engine/run_phase8_evaluation.py`.
