"""
data/synthetic_fog/generate_fog.py

Generates a synthetic FOG-grade IMU dataset at ~200 Hz based on real vehicle trajectories.
Applies published Fiber Optic Gyroscope (FOG) / Navigation-grade sensor characteristics:
- Ultra-low Angle Random Walk (ARW) ~ 0.005 deg / sqrt(hr)
- Minimal Gyro Bias Drift ~ 0.05 deg / hr
- Low Velocity Random Walk (VRW) ~ 0.02 mg / sqrt(Hz)
- Accelerometer Bias Instability ~ 20 ug

Explicitly labeled as SYNTHETIC data for Phase 12 validation (no claim of real FOG hardware).
"""

import os
import numpy as np
from scipy.interpolate import interp1d
import sys

# Add project root to PYTHONPATH for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from engine.calibration.calibrator import CalibrationEngine

def latlon_to_enu(lat, lon, alt, lat0, lon0, alt0):
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f**2
    dlat = np.radians(lat - lat0)
    dlon = np.radians(lon - lon0)
    dalt = alt - alt0
    lat0_rad = np.radians(lat0)

    R_N = a / np.sqrt(1 - e2 * np.sin(lat0_rad)**2)
    R_M = a * (1 - e2) / (1 - e2 * np.sin(lat0_rad)**2)**1.5

    e = dlon * (R_N + alt0) * np.cos(lat0_rad)
    n = dlat * (R_M + alt0)
    u = dalt
    return e, n, u

def generate_synthetic_fog_dataset(
    source_npz_path: str = "data/processed/S (Driver A)/S1/S1_synced.npz",
    output_npz_path: str = "data/synthetic_fog/s1_synthetic_fog_200hz.npz",
    target_freq: float = 200.0
):
    if not os.path.exists(source_npz_path):
        raise FileNotFoundError(f"Source file not found: {source_npz_path}")

    data = np.load(source_npz_path)
    time_orig = data['time']
    acc_raw = np.column_stack([data['acc_x'], data['acc_y'], data['acc_z']])
    gyro_raw = np.column_stack([data['gyro_x'], data['gyro_y'], data['gyro_z']])

    lat0 = data['gt_lat'][0]
    lon0 = data['gt_lon'][0]
    alt0 = data['gt_alt'][0]
    e_gt, n_gt, u_gt = latlon_to_enu(data['gt_lat'], data['gt_lon'], data['gt_alt'], lat0, lon0, alt0)
    gps_pos_orig = np.column_stack([e_gt, n_gt, u_gt])

    # GT velocity from speed and heading (or diff)
    speed = data['gt_speed']
    heading = np.radians(data['gt_heading'])
    vel_e = speed * np.sin(heading)
    vel_n = speed * np.cos(heading)
    vel_u = np.zeros_like(speed)
    gps_vel_orig = np.column_stack([vel_e, vel_n, vel_u])

    # 1. Calibrate smartphone data to Vehicle Frame (so it acts like rigidly mounted FOG)
    dt_orig = 0.1
    calib = CalibrationEngine()
    speed_nan_to_zero = np.nan_to_num(speed, nan=0.0)
    # Use first 120s for calibration
    calib.calibrate_from_session(acc_raw[:1200], gyro_raw[:1200], speed_nan_to_zero[:1200], dt=dt_orig)
    acc_orig, gyro_orig = calib.apply(acc_raw, gyro_raw)

    # Re-apply gravity perfectly to the Vehicle-Frame acceleration
    # so that the simulated fixed-frame FOG sensor sees gravity strictly along Z.
    # calib.apply removes gravity bias if it finds it, we want pure 9.80665 mostly in +Z.
    # Wait: accel in vehicle frame during stationary should be [0, 0, 9.80665].
    # The CalibrationEngine computes gravity, but it subtracts out the overall stationary bias.
    # We should add [0, 0, 9.80665] back to simulate what an aligned hardware sensor reads.
    acc_orig += np.array([0.0, 0.0, 9.80665])

    # Target time vector at 200 Hz
    dt = 1.0 / target_freq
    t_start = time_orig[0]
    t_end = time_orig[-1]
    time_200hz = np.arange(t_start, t_end, dt)

    # 2. Smooth kinematic interpolation to 200Hz
    interp_acc = interp1d(time_orig, acc_orig, axis=0, kind='cubic', fill_value="extrapolate")
    interp_gyro = interp1d(time_orig, gyro_orig, axis=0, kind='cubic', fill_value="extrapolate")
    interp_gps_pos = interp1d(time_orig, gps_pos_orig, axis=0, kind='linear', fill_value="extrapolate")
    interp_gps_vel = interp1d(time_orig, gps_vel_orig, axis=0, kind='linear', fill_value="extrapolate")

    acc_base = interp_acc(time_200hz)
    gyro_base = interp_gyro(time_200hz)
    gps_pos = interp_gps_pos(time_200hz)
    gps_vel = interp_gps_vel(time_200hz)

    # 3. Add realistic FOG noise characteristics
    n_samples = len(time_200hz)

    # Gyro parameters: ARW = 0.005 deg/sqrt(hr) -> rad/sqrt(s)
    gyro_white_noise_std = 1.45e-6 * np.sqrt(target_freq)
    gyro_bias_drift = np.cumsum(np.random.normal(0, 1e-8, (n_samples, 3)), axis=0) # ultra-stable bias
    gyro_noise = np.random.normal(0, gyro_white_noise_std, (n_samples, 3)) + gyro_bias_drift

    # Accel parameters: VRW = 0.02 mg/sqrt(Hz) -> m/s^2 / sqrt(Hz)
    acc_white_noise_std = 1.96e-4 * np.sqrt(target_freq)
    acc_bias_drift = np.cumsum(np.random.normal(0, 1e-7, (n_samples, 3)), axis=0)
    acc_noise = np.random.normal(0, acc_white_noise_std, (n_samples, 3)) + acc_bias_drift

    acc_fog = acc_base + acc_noise
    gyro_fog = gyro_base + gyro_noise

    # Save the synthetic dataset
    os.makedirs(os.path.dirname(output_npz_path), exist_ok=True)
    np.savez(
        output_npz_path,
        time=time_200hz,
        acc=acc_fog,
        gyro=gyro_fog,
        gps_pos=gps_pos,
        gps_vel=gps_vel,
        freq=target_freq,
        is_synthetic=True,
        description="Synthetic rigidly-mounted FOG-grade IMU dataset at 200Hz derived from S1 IO-VNBD trajectory."
    )
    print(f"Generated synthetic FOG dataset at {output_npz_path}: {n_samples} samples ({t_end - t_start:.2f} s @ {target_freq} Hz)")

if __name__ == "__main__":
    generate_synthetic_fog_dataset()
