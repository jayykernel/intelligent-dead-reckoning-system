# System Architecture Specification

**Project:** SIH PS 26168 — AI/ML based Intelligent Dead Reckoning System  
**Document Version:** 1.0.0  
**Date:** 2026-09-10  
**Status:** Approved for Implementation

---

## 1. Executive Architecture Overview

The system realizes a hybrid **Physics-Informed, Machine-Learning-Augmented Inertial Navigation Framework** designed for seamless transitions between GNSS-available and GNSS-denied environments.

```
RAW SENSORS (Accelerometer, Gyroscope, Magnetometer, GNSS/NavIC, Barometer)
                                    │
                                    ▼
                        SENSOR ABSTRACTION LAYER [Phase 2]
                                    │
                                    ▼
                           TIME SYNCHRONIZATION [Phase 2]
                                    │
                                    ▼
                            CALIBRATION ENGINE [Phase 3]
                    (Biases, Scale Factors, Hard/Soft Iron)
                                    │
                                    ▼
                         PHONE-TO-VEHICLE ALIGNMENT [Phase 4]
                       (Gravity + Velocity Dynamics)
                                    │
                  ┌─────────────────┴──────────────────┐
                  │                                    │
                  ▼                                    ▼
         MOTION INTELLIGENCE LAYER            DETERMINISTIC KINEMATICS [Phase 5]
       ┌────────────────────────────┐       ┌────────────────────────────┐
       │ - Temporal CNN / TCN-GRU   │       │ - Quaternion Attitude Int. │
       │   [Phase 10]               │       │ - Specific Force Transform │
       │ - Vehicle Classification   │       │ - Velocity & Position Prop │
       │   [FUTURE PHASE]           │       │ - Error Covariance Prop.   │
       │ - Stationary Prob P(ZUPT)  │       └──────────────┬─────────────┘
       │   [Phase 7]                │                      │
       │ - Vibration Energy Anal.   │                      │
       │   [Phase 9]                │                      │
       │ - Forward Vel & Var        │                      │
       │   [Phase 7/10]             │                      │
       │ - Magnetic Reliability     │                      │
       │   [FUTURE PHASE]           │                      │
       └──────────────┬─────────────┘                      │
                      │                                    │
                      └─────────────────┬──────────────────┘
                                        │
                                        ▼
                          ERROR-STATE KALMAN FILTER [Phase 6]
                              (15-State Error Core)
                                        │
                         ┌──────────────┴──────────────┐
                         ▼                             ▼
              ADAPTIVE CONSTRAINT ENGINE        MAP MATCHING ENGINE [FUTURE PHASE]
            ┌───────────────────────────┐     ┌────────────────────────────┐
            │ - Soft Adaptive ZUPT      │     │ - Multi-hypothesis graph   │
            │   [Phase 7]               │     │ - Road geometry & heading  │
            │ - Vehicle-Aware NHC       │     │ - Vertical/Level Disambig. │
            │   [FUTURE PHASE]          │     └────────────┬───────────────┘
            │ - Dynamic Covariance R    │                  │
            │   [Phase 8]               │                  │
            └─────────────┬─────────────┘                  │
                          │                                │
                          └──────────────┬─────────────────┘
                                         │
                                         ▼
                     NAVIGATION INTEGRITY ENGINE [FUTURE PHASE]
                    (Position, Velocity, Heading Covariances,
                     Integrity Levels, Degradation State Machine)
                                         │
                                         ▼
                         OUTPUT & API INTERFACE [FUTURE PHASE]
                 (Pose, Covariance, Confidence, Mode Diagnostics)
```

---

## 2. Mathematical Foundation & Coordinate Frames

### 2.1 Coordinate Systems

1. **Sensor Frame ($S$):** Raw physical tri-axis sensor coordinates attached to the smartphone/board.
2. **Body / Phone Frame ($B$):** Orthogonalized sensor body frame after factory misalignment and calibration corrections.
3. **Vehicle Frame ($V$):**
   - $X_v$: Forward (direction of vehicle motion).
   - $Y_v$: Right (lateral).
   - $Z_v$: Down (vertical, completing right-handed orthogonal system).
4. **Navigation Frame ($N$):** Local Cartesian Local-Level NED (North-East-Down) or East-North-Up (ENU) tangent plane referenced to WGS-84 origin $p_0 = (\phi_0, \lambda_0, h_0)$.
5. **Earth Frame ($E$):** Earth-Centered Earth-Fixed (ECEF) Cartesian coordinates for global geodetic mapping.

### 2.2 Stochastic Sensor Measurement Models

$$a_m(t) = a_{\text{true}}(t) + b_a(t) + S_a a_{\text{true}}(t) + n_a(t)$$
$$\omega_m(t) = \omega_{\text{true}}(t) + b_g(t) + S_g \omega_{\text{true}}(t) + n_g(t)$$
$$m_m(t) = C_{\text{softIron}} (m_{\text{true}}(t) + b_{\text{hardIron}}) + n_m(t)$$

Dynamic biases are modeled as first-order Gauss-Markov or random walk processes:
$$\dot{b}_a(t) = w_{ba}(t), \quad w_{ba} \sim \mathcal{N}(0, Q_{ba})$$
$$\dot{b}_g(t) = w_{bg}(t), \quad w_{bg} \sim \mathcal{N}(0, Q_{bg})$$

---

## 3. Error-State Kalman Filter (ESKF) Architecture

The nominal state uses a 16-parameter representation (including a quaternion) to track large-scale physical kinematics without singularity; the ESKF tracks small perturbations driven by linearized dynamics using a 15-DOF error state.

### 3.1 State Representation

**Nominal State ($x$):**
$$x = \begin{bmatrix} p^n \\ v^n \\ q_b^n \\ b_a \\ b_g \end{bmatrix} \in \mathbb{R}^{16}$$
- $p^n = [p_N, p_E, p_D]^T$: Position in navigation frame.
- $v^n = [v_N, v_E, v_D]^T$: Velocity in navigation frame.
- $q_b^n = [q_w, q_x, q_y, q_z]^T$: Unit quaternion representing orientation from body to navigation frame ($R_b^n(q)$).
- $b_a = [b_{ax}, b_{ay}, b_{az}]^T$: Accelerometer bias in body frame.
- $b_g = [b_{gx}, b_{gy}, b_{gz}]^T$: Gyroscope bias in body frame.

**Error State ($\delta x$):**
$$\delta x = \begin{bmatrix} \delta p^n \\ \delta v^n \\ \delta \theta^n \\ \delta b_a \\ \delta b_g \end{bmatrix} \in \mathbb{R}^{15}$$
where $\delta \theta^n$ represents the 3-axis rotation angle error vector: $q_b^n \approx \delta q \otimes \hat{q}_b^n$ with $\delta q \approx \begin{bmatrix} 1 \\ \frac{1}{2}\delta\theta^n \end{bmatrix}$.

### 3.2 Continuous-Time Error Dynamics

$$\delta \dot{p}^n = \delta v^n$$
$$\delta \dot{v}^n = -[R_b^n(\hat{a}_m - b_a)]_\times \delta\theta^n - R_b^n \delta b_a - R_b^n n_a$$
$$\delta \dot{\theta}^n = -[\omega_{\text{in}}^n]_\times \delta\theta^n - R_b^n \delta b_g - R_b^n n_g$$
$$\delta \dot{b}_a = w_{ba}$$
$$\delta \dot{b}_g = w_{bg}$$

where $[v]_\times$ denotes the skew-symmetric cross-product matrix:
$$[v]_\times = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}$$

### 3.3 Discrete-Time Propagation

Given sampling interval $\Delta t = t_{k} - t_{k-1}$:

**Nominal State Propagation:**
$$a_{\text{unbiased}} = a_m - b_a$$
$$\omega_{\text{unbiased}} = \omega_m - b_g$$
$$q_{k} = q_{k-1} \otimes \Delta q(\omega_{\text{unbiased}} \Delta t)$$
$$a^n = R(q_k) a_{\text{unbiased}} + g^n$$
$$v^n_k = v^n_{k-1} + a^n \Delta t$$
$$p^n_k = p^n_{k-1} + v^n_{k-1} \Delta t + \frac{1}{2} a^n \Delta t^2$$

**Error State Transition Matrix ($F_k$):**
$$F_k = \begin{bmatrix}
I_{3\times 3} & I_{3\times 3}\Delta t & -\frac{1}{2}[R(q_k)a_{\text{unbiased}}]_\times \Delta t^2 & -\frac{1}{2}R(q_k)\Delta t^2 & 0_{3\times 3} \\
0_{3\times 3} & I_{3\times 3} & -[R(q_k)a_{\text{unbiased}}]_\times \Delta t & -R(q_k)\Delta t & 0_{3\times 3} \\
0_{3\times 3} & 0_{3\times 3} & I_{3\times 3} - [\omega_{\text{unbiased}}]_\times \Delta t & 0_{3\times 3} & -R(q_k)\Delta t \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & I_{3\times 3} & 0_{3\times 3} \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & I_{3\times 3}
\end{bmatrix}$$

**Covariance Propagation:**
$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k$$

### 3.4 Measurement Update and State Reset

For any measurement $z_m$ with measurement model $h(x)$, innovation $y = z_m - h(\hat{x}_{k|k-1})$ and Jacobian $H = \left.\frac{\partial h}{\partial \delta x}\right|_{\hat{x}}$:

$$S_k = H_k P_{k|k-1} H_k^T + R_k$$
$$K_k = P_{k|k-1} H_k^T S_k^{-1}$$
$$\delta \hat{x} = K_k y$$
$$P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T \quad \text{(Joseph Form)}$$

**State Injection & Error Reset:**
$$\hat{p}^n \leftarrow \hat{p}^n + \delta\hat{p}^n$$
$$\hat{v}^n \leftarrow \hat{v}^n + \delta\hat{v}^n$$
$$\hat{q}_b^n \leftarrow \Delta q(\delta\hat{\theta}^n) \otimes \hat{q}_b^n, \quad \hat{q}_b^n \leftarrow \frac{\hat{q}_b^n}{\|\hat{q}_b^n\|}$$
$$\hat{b}_a \leftarrow \hat{b}_a + \delta\hat{b}_a$$
$$\hat{b}_g \leftarrow \hat{b}_g + \delta\hat{b}_g$$
$$\delta x \leftarrow 0_{15\times 1}$$

---

## 4. Phone-to-Vehicle Alignment Engine

Smartphones are placed arbitrarily inside vehicles. The rotation matrix $R_p^v$ relating Phone frame ($p$) to Vehicle frame ($v$) is estimated continuously:

$$R_b^n = R_v^n R_p^v$$

### 4.1 Two-Phase Dynamic Alignment

1. **Coarse Pitch/Roll Alignment (Gravity Vector):**
   During stationary or constant-velocity intervals:
   $$\bar{a}^p \approx -R_v^p g^v = \begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix}$$
   The down-axis of the vehicle frame corresponds to normalized acceleration:
   $$z_v^p = -\frac{\bar{a}^p}{\|\bar{a}^p\|}$$

2. **Fine Yaw Alignment (Dynamic Forward Acceleration / GNSS Heading):**
   When vehicle accelerates longitudinally ($\dot{v} > 0.5\text{ m/s}^2$) or maintains stable GNSS track velocity $v_{\text{gnss}}^n$:
   $$x_v^p = \text{normalize}\left(\frac{\Delta v^p}{\Delta t} - (\frac{\Delta v^p}{\Delta t} \cdot z_v^p) z_v^p\right)$$
   $$y_v^p = z_v^p \times x_v^p$$
   $$R_p^v = \begin{bmatrix} (x_v^p)^T \\ (y_v^p)^T \\ (z_v^p)^T \end{bmatrix}$$

3. **Continuous Misalignment Tracking & Disturbance Detection:**
   Monitor residual angular velocity norm and high-rate acceleration variance. If $\|\omega^p - \omega^v\| > \epsilon_{\text{slip}}$, flag a phone reorientation event, decay alignment confidence, and re-initiate estimation.

---

## 5. Motion Intelligence & ML Subsystem

The ML layer provides soft probabilistic indicators and continuous velocity constraints rather than directly predicting coordinates.

```
IMU Window (N=100-200, 10-200 Hz)
   [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_norm]
                           │
                           ▼
          FEATURE EXTRACTION & PREPROCESSING
     - Specific force magnitude, dynamic variance
     - Fast Fourier Transform (FFT) spectral bands (0-5Hz, 5-15Hz, 15-50Hz)
     - Spectral entropy, zero-crossing rates
                           │
                           ▼
         TEMPORAL CONVOLUTIONAL NETWORK (TCN)
               / LIGHTWEIGHT GRU CORE
         ┌─────────────────┬─────────────────┐
         │                 │                 │
         ▼                 ▼                 ▼
   HEAD 1: REGRESSION   HEAD 2: CLASSIF.  HEAD 3: ANOMALY
   - Forward Velocity   - Motion State    - Vibration Index
     $\hat{v}_{\text{fwd}}$ (Stationary,    - Magnetic Rel.
   - Uncertainty          Const, Turn,      $P(\text{mag\_ok})$
     $\sigma_v^2$         Accel, Decel)   - Vehicle Type
                        - $P(\text{ZUPT})$  (Car, Bike, Scoot)
```

### 5.1 Model Specifications
- **Input Dimension:** $T \times 7$ (e.g., $100 \text{ frames} \times [a_x, a_y, a_z, \omega_x, \omega_y, \omega_z, \|m\|]$)
- **Architecture Options:**
  - *Option A (Edge/Mobile):* 4-layer Dilated Temporal Convolutional Network (TCN) with residual connections. Kernel size $k=3$, dilations $d \in \{1, 2, 4, 8\}$, channel width 32.
  - *Option B (Alternative):* 2-layer Bidirectional GRU (hidden size 64) with temporal attention pooling.
- **Inference Latency Target:** $< 5 \text{ ms}$ on mobile CPU via TFLite / ONNX Runtime.

---

## 6. Adaptive Physical Constraints Engine

### 6.1 Soft Zero Velocity Update (Soft ZUPT)
Unlike hard thresholding, measurement covariance $R_{\text{zupt}}$ scales inversely with stationary probability $P_{\text{stat}}$:

$$z_{\text{zupt}} = \hat{v}^n - 0_{3\times 1}$$
$$H_{\text{zupt}} = \begin{bmatrix} 0_{3\times 3} & I_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \end{bmatrix}$$
$$R_{\text{zupt}} = \text{diag}\left( \frac{\sigma_{\text{zupt, base}}^2}{P_{\text{stat}}^\gamma + \epsilon} \right)$$
where $\gamma \ge 2$ penalizes uncertain detections.

### 6.2 Vehicle-Aware Non-Holonomic Constraints (NHC) [APPROVED FUTURE DESIGN]
Under non-slipping land vehicle dynamics, lateral velocity $v_y^v \approx 0$ and vertical velocity $v_z^v \approx 0$:

$$v^v = R_n^v v^n = (R_p^v R_b^p R_n^b) v^n$$
$$z_{\text{nhc}} = \begin{bmatrix} v_y^v \\ v_z^v \end{bmatrix} = \begin{bmatrix} (y_v^n)^T v^n \\ (z_v^n)^T v^n \end{bmatrix} \approx \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$
$$H_{\text{nhc}} = \begin{bmatrix} 0_{2\times 3} & \begin{bmatrix} (y_v^n)^T \\ (z_v^n)^T \end{bmatrix} & 0_{2\times 3} & 0_{2\times 3} & 0_{2\times 3} \end{bmatrix}$$

**Vehicle-Adaptive Scaling:**
- **Car / 4-Wheeler:** $R_{\text{nhc}} = \text{diag}(\sigma_{\text{lat}}^2, \sigma_{\text{vert}}^2)$ with high rigidity $(\sigma_{\text{lat}} \approx 0.05 \text{ m/s})$.
- **Motorcycle / Scooter:** Account for roll lean angle $\phi_{\text{lean}}$ and yaw rate $\dot{\psi}$:
  $$R_{\text{nhc, bike}} = \text{diag}\left( \sigma_{\text{lat}}^2 \cdot (1 + \kappa_1 \sin^2\phi_{\text{lean}} + \kappa_2 |\dot{\psi}|), \sigma_{\text{vert}}^2 \right)$$
  When turning or leaning, lateral constraint is automatically relaxed.

### 6.3 ML Forward Velocity Update
$$z_{\text{v\_ml}} = x_v^n \cdot v^n - \hat{v}_{\text{fwd}}$$
$$R_{\text{v\_ml}} = \sigma_{\text{v\_pred}}^2$$

---

## 7. GNSS Integrity & Seamless Blackout Transition Engine

```
       ┌────────────────────────────────────────────────┐
       │               GNSS + INS NORMAL                │
       └───────────────────────┬────────────────────────┘
                               │ GNSS Accuracy degraded / Outage detected
                               ▼
       ┌────────────────────────────────────────────────┐
       │             TRANSITION TO DEAD RECKONING       │
       │ - Latch last known reliable biases & alignment │
       │ - Inflate process noise Q dynamically          │
       │ - Activate ML velocity + Adaptive NHC + ZUPT   │
       └───────────────────────┬────────────────────────┘
                               │ Outage ongoing
                               ▼
       ┌────────────────────────────────────────────────┐
       │             INTELLIGENT DEAD RECKONING         │
       │ - Pure inertial mechanization + ML updates     │
       │ - Map matching multi-hypothesis bounding       │
       │ - Continuous covariance growth tracking        │
       └───────────────────────┬────────────────────────┘
                               │ GNSS signal returns
                               ▼
       ┌────────────────────────────────────────────────┐
       │             TRANSITION TO GNSS RECOVERY        │
       │ - Innovation gating: $y^T S^{-1} y < \gamma$   │
       │ - Smooth gain ramping (avoid step jumps)       │
       │ - Bias re-convergence                         │
       └───────────────────────┬────────────────────────┘
                               │ Normal tracking established
                               ▼
       ┌────────────────────────────────────────────────┐
       │               GNSS + INS NORMAL                │
       └────────────────────────────────────────────────┘
```

---

## 8. Multi-Hypothesis Map Reasoning [APPROVED FUTURE DESIGN]

When operating in dense urban environments, multi-level structures, or complex highway interchanges:
1. **Hypothesis Generation:** Candidate road segments within $3\sigma$ position error ellipse.
2. **Scoring Function:**
   $$\text{Score}(h_i) = w_d \cdot \exp\left(-\frac{d_{\perp}^2}{2\sigma_d^2}\right) + w_\theta \cdot \cos(\psi_{\text{est}} - \psi_{\text{road}}) + w_z \cdot \exp\left(-\frac{\Delta h_{\text{baro}}^2}{2\sigma_h^2}\right) + w_{\text{top}}\cdot T_{i, i-1}$$
3. **Multi-Level Parking Disambiguation:** Uses calibrated barometric pressure $\Delta P = -\rho g \Delta h$ combined with helical yaw pattern detection (spiral ramp ascent/descent) to disambiguate floor level $L = \text{round}(\Delta h / h_{\text{floor}})$.

---

## 9. Navigation Integrity & Uncertainty Reporting [APPROVED FUTURE DESIGN]

The system outputs a comprehensive, non-fabricated telemetry object at 10 Hz / 200 Hz. *(Note: The JSON object below is an **ILLUSTRATIVE EXAMPLE** of the target output, not currently measured output)*:

```json
{
  "timestamp_ns": 1725969600000000000,
  "latitude_deg": 12.9715987,
  "longitude_deg": 77.5945627,
  "altitude_m": 920.45,
  "velocity_ned_mps": [12.4, 0.2, -0.05],
  "speed_mps": 12.401,
  "heading_deg": 88.5,
  "uncertainty": {
    "position_std_m": [1.2, 1.4, 3.1],
    "horizontal_error_radius_95_m": 2.85,
    "velocity_std_mps": [0.15, 0.12, 0.25],
    "heading_std_deg": 1.8
  },
  "motion_state": {
    "vehicle_class": "CAR",
    "vehicle_class_prob": 0.96,
    "is_stationary": false,
    "stationary_prob": 0.002,
    "vibration_level": "MODERATE",
    "magnetic_reliable": false
  },
  "alignment": {
    "status": "CONVERGED",
    "pitch_deg": 2.1,
    "roll_deg": -1.4,
    "yaw_offset_deg": 14.2,
    "confidence": 0.94
  },
  "navigation_mode": "INTELLIGENT_DR",
  "integrity_level": "HIGH_CONFIDENCE",
  "diagnostics": {
    "gnss_outage_duration_s": 24.5,
    "accumulated_drift_ratio_percent": 0.68,
    "active_constraints": ["NHC_CAR", "ML_VELOCITY"]
  }
}
```

---

## 10. Modular Software Architecture

```
dead-reckoning-core/
├── apps/
│   ├── android/              # Native Android Kotlin App (Sensors, UI, MapView)
│   ├── edge_daemon/          # C++/Python 200 Hz Edge Engine
│   └── web_dashboard/        # Real-time WebSocket Telemetry UI
├── core/
│   ├── sensors/              # Sensor Abstraction Layer, Types, RingBuffer
│   ├── calibration/          # Online/Offline Sensor Calibration
│   ├── alignment/            # Phone-to-Vehicle Dynamic Estimator
│   ├── navigation/           # Mechanization, Kinematics, Frames
│   ├── filters/              # ESKF, Covariance Managers, Jacobians
│   ├── constraints/          # Soft ZUPT, Adaptive NHC, ML-Velocity
│   ├── motion/               # Vibration Engine, Motion Feature Extractor
│   ├── models/               # TCN / GRU Architectures, ONNX/TFLite Run
│   ├── gnss/                 # Quality Monitor, Outage Manager, Transitions
│   ├── map/                  # Road Graph, Map Matcher, Vertical Resolver
│   └── integrity/            # Error Propagation, Integrity State Machine
├── tools/
│   ├── replay/               # Deterministic Sensor Stream Replayer
│   ├── simulator/            # Synthetic IMU/GNSS Generator & Blackout Injector
│   ├── evaluation/           # Trajectory Metrics (ATE, RTE, Drift Ratio)
│   └── ablation/             # Automated Multi-Variant Benchmark Runner
└── docs/                     # Architecture, Audits, Mathematical References
```

---

## 11. Verification and Performance Standards (ENGINEERING REQUIREMENTS / TARGETS)

| Metric | Minimum Acceptable | Target Objective | Validation Protocol |
|---|---|---|---|
| **60s GNSS Outage Drift** | $\le 2.5\%$ distance | $\le 1.0\%$ distance | Replay on IO-VNBD & Field Data |
| **ZUPT Detection Accuracy** | $\ge 95\%$ | $\ge 99\%$ (zero false ZUPT during cruise) | Benchmarking on Stop-and-Go Logs |
| **Inference Latency (Mobile)** | $\le 15\text{ ms}$ | $\le 5\text{ ms}$ | Android Benchmark (Snapdragon 7/8 series) |
| **Edge Pipeline Frequency** | $\ge 100\text{ Hz}$ | $\ge 200\text{ Hz}$ | POSIX Edge Daemon Benchmark |
| **Phone Reorientation Recov.** | $\le 5.0\text{ s}$ | $\le 2.0\text{ s}$ | Dynamic Perturbation Tests |
| **Vehicle Class Accuracy** | $\ge 85\%$ | $\ge 94\%$ | Cross-vehicle Tri-Class Test Split |
