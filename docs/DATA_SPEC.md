# IO-VNBD Dataset Documentation

## Overview
The **Intelligent Operations - Vehicle Navigation Benchmark Dataset (IO-VNBD)** is an automotive dataset collecting synchronized smartphone sensor data and vehicle telemetry across multiple drivers and routes.

## Directory Layout
Raw data is located in `data/raw/Categorised IOVNB Dataset/` and organized by driver and session:
- `S (Driver A)/`: Sessions S1, S2, S3a, S3b, S3c, S4 (6 sessions)
- `M (Driver B)/`: Session M (1 session)
- `Y (Driver D)/`: Session Y1 (1 session)
- `Vf (Driver E)/`: Sessions V-Vfa01, V-Vfa02 (2 sessions)
- `Vta (Driver E)/`: Sessions Vta01a to Vta30 (30 sessions)
- `Vtb (Driver E)/`: Sessions Vtb01 to Vtb12 (12 sessions)
- `Vw (Driver E)/`: Sessions Vw01 to Vw17 (20 sessions)

Total: **72 recorded sessions**.

## File Formats & Field Definitions

Each session contains two primary CSV files:

### 1. Smartphone Sensor Data (`S-*.csv`)
Sampled nominally at 10 Hz (approx. 100 ms interval).

| Field Name | Unit | Description | Coordinate Frame |
|---|---|---|---|
| `GPS LATITUDE (degrees)` | degrees | Smartphone GPS latitude | WGS84 |
| `GPS LONGITUDE (degrees)` | degrees | Smartphone GPS longitude | WGS84 |
| `GPS ALTITUDE (m)` | meters | Smartphone GPS altitude | Above sea level |
| `GPS SPEED (Kmh)` | km/h | Smartphone GPS estimated ground speed | Scaled scalar |
| `GPS ACCURACY (m)` | meters | Smartphone GPS horizontal accuracy estimate | 1-sigma circle |
| `GPS ORIENTATION (°)` | degrees | Smartphone GPS bearing/heading | True North (0-360) |
| `GPS SATELLITES IN RANGE`| count / count | Satellites used vs satellites in view | N/A |
| `TIME SINCE START (ms)` | ms | Milliseconds since start of recording | Monotonic timestamp |
| `DATE (YYYY-MO-DD...)` | ISO string | Timestamp | Local/UTC |
| `ACCELEROMETER X (m/s²)`| m/s² | Specific force along Phone X-axis (Right) | Phone Body Frame |
| `ACCELEROMETER Y (m/s²)`| m/s² | Specific force along Phone Y-axis (Up/Forward) | Phone Body Frame |
| `ACCELEROMETER Z (m/s²)`| m/s² | Specific force along Phone Z-axis (Out of screen) | Phone Body Frame |
| `GRAVITY X (m/s²)` | m/s² | Estimated gravity component X | Phone Body Frame |
| `GRAVITY Y (m/s²)` | m/s² | Estimated gravity component Y | Phone Body Frame |
| `GRAVITY Z (m/s²)` | m/s² | Estimated gravity component Z | Phone Body Frame |
| `GYROSCOPE Yaw (rad/s)` | rad/s | Angular rate around Z-axis | Phone Body Frame |
| `GYROSCOPE Pitch (rad/s)`| rad/s | Angular rate around X-axis | Phone Body Frame |
| `GYROSCOPE Roll (rad/s)` | rad/s | Angular rate around Y-axis | Phone Body Frame |
| `MAGNETIC FIELD X (μT)` | μT | Calibrated magnetic field X | Phone Body Frame |
| `MAGNETIC FIELD Y (μT)` | μT | Calibrated magnetic field Y | Phone Body Frame |
| `MAGNETIC FIELD Z (μT)` | μT | Calibrated magnetic field Z | Phone Body Frame |
| `ORIENTATION (Yaw) (°)` | degrees | Phone orientation yaw (Euler) | Body-to-World |
| `ORIENTATION (Pitch) (°)`| degrees | Phone orientation pitch (Euler) | Body-to-World |
| `ORIENTATION (Roll) (°)` | degrees | Phone orientation roll (Euler) | Body-to-World |

### 2. Vehicle Ground Truth & Telemetry (`V-*.csv`)
Synchronized reference data from high-precision vehicle CAN/OBD system and reference GPS.

| Field Name | Unit | Description |
|---|---|---|
| `Time Since Start of Day (seconds)` | seconds | High-precision synchronized time |
| `Latitude (degrees)` | degrees | Reference GPS Latitude (Ground Truth) |
| `Longitude (degrees)` | degrees | Reference GPS Longitude (Ground Truth) |
| `Velocity (km/hr)` | km/h | Reference vehicle ground speed |
| `Heading (degrees)` | degrees | Reference vehicle heading (True North) |
| `Height (km)` | km | Reference altitude |
| `Indicated Vehicle Speed (km/hr)` | km/h | Wheel-speed-derived vehicle speed |
| `Indicated Longitudinal Acceleration (g)` | g | Vehicle longitudinal acceleration (1g = 9.80665 m/s²) |
| `Indicated Lateral Acceleration (g)` | g | Vehicle lateral acceleration |
| `Wheel Speed Front Left/Right (rad/s)` | rad/s | Individual wheel angular velocities |
| `Yaw Rate (deg/sec)` | deg/s | Reference vehicle yaw rate |

## Coordinate Systems
- **Phone Body Frame**:
  - +X: Right edge of phone screen
  - +Y: Top edge of phone screen
  - +Z: Orthogonal out of the screen (towards user)
- **Vehicle Frame (Standard SAE / ISO 8855)**:
  - +X: Forward along vehicle longitudinal axis
  - +Y: Left (or Right depending on convention; standard ENU vehicle frame has +X Forward, +Y Left, +Z Up)
  - +Z: Upwards perpendicular to road surface
- **Navigation Frame**:
  - Local East-North-Up (ENU) tangent plane relative to reference origin.

## Session Characteristics (for baseline interpretation)

### Session S1 (Driver A)
| Metric | Value |
|--------|-------|
| Total Duration | 5174.4 s (86.2 min) |
| Average Speed | 7.33 m/s (26.40 km/h) |
| Max Speed | 26.07 m/s (93.83 km/h) |
| Stationary Time (<0.5 m/s) | 11.1% |
| Total Distance | 37,948.4 m |
| Driving Pattern | Mixed urban/suburban with frequent stops |

**60-second baseline window** (used in Phase 2 drift measurement):
| Metric | Value |
|--------|-------|
| Average Speed | 1.55 m/s (5.56 km/h) |
| Stationary Time | 72.3% |
| Distance | 92.7 m |
| Path Length | 96.82 m |

*Note: The high stationary percentage (72.3%) in the 60-second window reflects initial idle departure, making it a challenging baseline for velocity estimation. The Phase 2 drift of 2148.91% is measured against this low-motion segment.*
