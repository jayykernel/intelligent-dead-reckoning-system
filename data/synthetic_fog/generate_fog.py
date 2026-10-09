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
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import numpy as np
from scipy.interpolate import interp1d
from engine.config.constants import GRAVITY_MS2
from training.data_loader import latlon_to_enu

def generate_synthetic_fog_dataset(
    source_npz_path: str = "data/processed/S (Driver A)/S1/S1_synced.npz",
    output_npz_path: str = "data/processed/s1_synthetic_fog_200hz.npz",
    target_freq: float = 200.0
):
    if not os.path.exists(source_npz_path):
        raise FileNotFoundError(f"Source file not found: {source_npz_path}")

    data = np.load(source_npz_path)
    time_orig = data['time']
    heading_deg = data['gt_heading']

    lat0 = data['gt_lat'][0]
    lon0 = data['gt_lon'][0]
    alt0 = data['gt_alt'][0] / 1000.0
    e_gt, n_gt, u_gt = latlon_to_enu(data['gt_lat'], data['gt_lon'], data['gt_alt'] / 1000.0, lat0, lon0, alt0)

    dt = 1.0 / target_freq
    t_start = time_orig[0]
    t_end = time_orig[-1]
    time_200hz = np.arange(t_start, t_end, dt)

    # 1. Smooth kinematic interpolation to 200Hz
    e_200 = interp1d(time_orig, e_gt, kind='cubic', fill_value="extrapolate")(time_200hz)
    n_200 = interp1d(time_orig, n_gt, kind='cubic', fill_value="extrapolate")(time_200hz)
    u_200 = interp1d(time_orig, u_gt, kind='cubic', fill_value="extrapolate")(time_200hz)

    # Unwrap heading to prevent discontinuities during differentiation
    head_200_unwrapped = np.unwrap(np.radians(interp1d(time_orig, heading_deg, kind='linear', fill_value="extrapolate")(time_200hz)))

    # Kinematics in ENU
    v_e = np.gradient(e_200, dt)
    v_n = np.gradient(n_200, dt)
    v_u = np.gradient(u_200, dt)

    a_e = np.gradient(v_e, dt)
    a_n = np.gradient(v_n, dt)
    a_u = np.gradient(v_u, dt)

    # Specific force in Nav frame (f = a - g, where g = [0, 0, -GRAVITY_MS2])
    f_e = a_e
    f_n = a_n
    f_u = a_u + GRAVITY_MS2

    # 2. Convert to vehicle frame
    # Vehicle X = Right, Y = Forward, Z = Up
    # Heading psi is CW from North
    psi = head_200_unwrapped
    f_veh_x = np.cos(psi)*f_e - np.sin(psi)*f_n
    f_veh_y = np.sin(psi)*f_e + np.cos(psi)*f_n
    f_veh_z = f_u

    dpsi_dt = np.gradient(psi, dt)
    gyro_x = np.zeros_like(dpsi_dt)
    gyro_y = np.zeros_like(dpsi_dt)
    gyro_z = -dpsi_dt  # since heading is CW, angular velocity around +Z is -dpsidt

    # 3. Add realistic FOG noise characteristics
    n_samples = len(time_200hz)

    # Gyro parameters: ARW = 0.005 deg/sqrt(hr)
    gyro_white_noise_std = 1.45e-6 * np.sqrt(target_freq)
    gyro_bias_drift = np.cumsum(np.random.normal(0, 1e-9, (n_samples, 3)), axis=0)
    gyro_noise = np.random.normal(0, gyro_white_noise_std, (n_samples, 3)) + gyro_bias_drift

    # Accel parameters: VRW = 0.02 mg/sqrt(Hz)
    acc_white_noise_std = 1.96e-4 * np.sqrt(target_freq)
    acc_bias_drift = np.cumsum(np.random.normal(0, 1e-8, (n_samples, 3)), axis=0)
    acc_noise = np.random.normal(0, acc_white_noise_std, (n_samples, 3)) + acc_bias_drift

    acc_fog = np.column_stack([f_veh_x, f_veh_y, f_veh_z]) + acc_noise
    gyro_fog = np.column_stack([gyro_x, gyro_y, gyro_z]) + gyro_noise

    gps_pos = np.column_stack([e_200, n_200, u_200])
    gps_vel = np.column_stack([v_e, v_n, v_u])

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