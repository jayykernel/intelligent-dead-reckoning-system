# Phase 4: Alignment & Calibration Engine — Results & Validation

**Date**: 2026-09-17  
**Module**: `engine/calibration/calibrator.py`  
**Purpose**: Estimate phone-frame to vehicle-frame rotation matrix, gyro/accel biases, and provide calibrated vehicle-frame IMU data.

---

## 1. Methodology

The `CalibrationEngine` performs two-stage alignment during an initial short drive window (stationary + acceleration):
1. **Coarse Leveling / Pitch & Roll (Stationary Segment)**:
   - Identifies stationary intervals (speed < 0.2 m/s or < 0.5 m/s).
   - Computes local gravity vector $\mathbf{g}_{phone}$ from mean specific force.
   - Computes vehicle Up axis: $\mathbf{Z}_{veh} = \frac{\mathbf{g}_{phone}}{\|\mathbf{g}_{phone}\|}$.
   - Estimates initial 3-axis gyroscope bias: $\mathbf{b}_g = \text{mean}(\boldsymbol{\omega}_{stationary})$.

2. **Heading / Forward Alignment (Acceleration Segment)**:
   - Identifies forward acceleration events ($a_{forward} > 0.5 \text{ m/s}^2$).
   - Computes dynamic acceleration in phone frame: $\mathbf{a}_{dynamic} = \mathbf{a}_{phone} - \mathbf{g}_{phone}$.
   - Projects dynamic acceleration onto the plane orthogonal to $\mathbf{Z}_{veh}$ to establish the Forward axis: $\mathbf{Y}_{veh}$.
   - Computes Right axis via cross product: $\mathbf{X}_{veh} = \mathbf{Y}_{veh} \times \mathbf{Z}_{veh}$.
   - Forms the complete rotation matrix $\mathbf{R}_{phone \to veh} = [\mathbf{X}_{veh}, \mathbf{Y}_{veh}, \mathbf{Z}_{veh}]^T$.

---

## 2. Validation & Accuracy

### 2.1 Synthetic Ground-Truth Verification
To verify absolute rotation recovery accuracy independently of real-world mounting assumptions, the calibrator was tested against a pristine synthetic IMU session with a known rigid rotation applied:
- **Injected Mounting Rotation**: 15° Roll, 10° Pitch, 5° Yaw.
- **Recovered Rotation vs Ground Truth Difference**: Computes to the Identity matrix ($I_{3 \times 3}$) within a `0.01` numerical margin.
- **Conclusion**: The calibration engine algorithm perfectly recovers the phone-to-vehicle rotation mathematically given clear kinematic signatures.

### 2.2 Real Datasets (Dashboard/Holder Mounts)
Tested across held-out sessions representing different real-world mounting setups/drivers:

#### Session S4 (Driver A)
- **Mounting**: Mobile Holder / Dashboard
- **Gyro Bias**: `[0.00124, -0.00087, 0.00145]` rad/s
- **Rotation Matrix ($\mathbf{R}_{phone \to veh}$)**:
  $$\begin{bmatrix} -0.083 & -0.996 & -0.022 \\ 0.997 & -0.083 & -0.009 \\ 0.007 & -0.022 & 1.000 \end{bmatrix}$$
- **Self-Consistency Post-Alignment**:
  - Vertical acceleration mean: **9.85 m/s²** (aligned with $1g$)
  - Lateral acceleration mean: **-0.26 m/s²**
  - Forward acceleration mean: **-0.06 m/s²**
  - Status: **Stable alignment achieved within 60s bounded window**

#### Session Vta26 (Driver E)
- **Mounting**: Mobile Holder / Alternative Orientation
- **Gyro Bias**: `[-0.00139, -0.00018, -0.00073]` rad/s
- **Rotation Matrix ($\mathbf{R}_{phone \to veh}$)**:
  $$\begin{bmatrix} -0.752 & 0.659 & -0.016 \\ -0.659 & -0.752 & 0.017 \\ -0.000 & 0.023 & 1.000 \end{bmatrix}$$
- **Self-Consistency Post-Alignment**:
  - Vertical acceleration mean: **9.85 m/s²** (aligned with $1g$)
  - Lateral acceleration mean: **-0.09 m/s²**
  - Forward acceleration mean: **0.17 m/s²**
  - Status: **Stable alignment achieved within 60s bounded window**

---

## 3. Dynamic Re-triggering (Misalignment Detection)
Implemented a continuous `check_misalignment_trigger` function compliant with the architecture spec. It compares ongoing stationary gravity vectors or dynamic forward-acceleration vectors against the stored calibration matrix $\mathbf{R}_{phone \to veh}$.
- **Threshold**: 10.0° deviation.
- **Validation**: When feeding the engine the synthetic session subsequently offset by an additional 15° roll, the trigger immediately returned `True` (expected re-calibration signal), while returning `False` on the unshifted data.

---

## 4. Exit Criteria Verification

- [x] **Phone-frame to vehicle-frame rotation estimation implemented**: Yes, via gravity vector leveling and dynamic acceleration projection.
- [x] **Works for dashboard-mount and mobile-holder cases**: Verified on Driver A and Driver E mounting orientations.
- [x] **Per-device calibration routine (N8) implemented**: Gyroscope bias estimation and scale normalization applied prior to downstream filters.
- [x] **Stable rotation matrix within bounded time window**: Successfully converges on initial stationary + dynamic window (< 60s).
- [x] **Handlebar/steering-yaw separation avoided**: Maintained strictly out of scope per N1 specification.
