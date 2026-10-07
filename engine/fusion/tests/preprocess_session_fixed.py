import os
import sys
import pandas as pd
import numpy as np
from datetime import datetime

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session

def preprocess_session_fixed(s_df, v_df, target_dt=0.1):
    """
    Clean, synchronize, and resample raw smartphone and vehicle telemetry.

    Output columns:
    - time: timestamp in seconds starting from 0.0 (absolute time alignment)
    - acc_x, acc_y, acc_z: phone specific force (m/s^2)
    - gyro_x, gyro_y, gyro_z: phone angular rates (rad/s)
    - mag_x, mag_y, mag_z: magnetic field (uT)
    - phone_lat, phone_lon, phone_alt: phone GPS position
    - phone_speed: phone GPS speed (m/s)
    - phone_heading: phone GPS orientation (deg)
    - gt_lat, gt_lon, gt_alt: ground truth position
    - gt_speed: ground truth forward velocity (m/s)
    - gt_heading: ground truth heading (deg)
    """
    # 1. Parse absolute start times from DATE strings
    def get_start_time(df, time_col, date_col):
        """Get absolute start time in seconds for a dataset"""
        first_date_str = df[date_col].iloc[0]
        time_part = first_date_str.split()[1]  # Get the time part
        # Replace colons with hyphens to parse as YYYY-MM-DD HH-MM-SS-SSS
        time_part = time_part.replace(':', '-')
        parts = time_part.split('-')
        h, m, s, ms = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
        return h * 3600 + m * 60 + s + ms / 1000.0

    s_start_sec = get_start_time(s_df, 'TIME SINCE START (ms)', 'DATE (YYYY-MO-DD HH-MI-SS_SSS)')
    v_start_sec = get_start_time(v_df, 'Time Since Start of Day (seconds)', 'DATE (YYYY-MO-DD HH-MI-SS_SSS)')

    # Calculate time offset between datasets
    time_offset = s_start_sec - v_start_sec  # Positive means s starts later

    print(f"Start time offset: s_df starts at {s_start_sec:.3f}s, v_df starts at {v_start_sec:.3f}s, offset = {time_offset:.3f}s")

    # 2. Normalize timestamps to seconds relative to start
    # Smartphone time is in ms
    s_time = (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0

    # Vehicle time is in seconds
    v_time = v_df['Time Since Start of Day (seconds)'] - v_start_sec  # Fixed: use v_start_sec instead of v_df.iloc[0]

    # 3. Find common duration
    max_duration = min(s_time.iloc[-1], v_time.iloc[-1])
    uniform_time = np.arange(0.0, max_duration, target_dt)

    # 4. Extract and interpolate sensor features
    # Gyro columns: IO-VNBD uses Yaw, Pitch, Roll in rad/s
    acc_x = np.interp(uniform_time, s_time, s_df['ACCELEROMETER X (m/s)'] if 'ACCELEROMETER X (m/s)' in s_df else s_df.filter(like='ACCELEROMETER X').iloc[:, 0])
    acc_y = np.interp(uniform_time, s_time, s_df['ACCELEROMETER Y (m/s)'] if 'ACCELEROMETER Y (m/s)' in s_df else s_df.filter(like='ACCELEROMETER Y').iloc[:, 0])
    acc_z = np.interp(uniform_time, s_time, s_df['ACCELEROMETER Z (m/s)'] if 'ACCELEROMETER Z (m/s)' in s_df else s_df.filter(like='ACCELEROMETER Z').iloc[:, 0])

    gyro_yaw = np.interp(uniform_time, s_time, s_df['GYROSCOPE Yaw (rad/s)'])
    gyro_pitch = np.interp(uniform_time, s_time, s_df['GYROSCOPE Pitch (rad/s)'])
    gyro_roll = np.interp(uniform_time, s_time, s_df['GYROSCOPE Roll (rad/s)'])

    mag_x = np.interp(uniform_time, s_time, s_df['MAGNETIC FIELD X (μT)'] if 'MAGNETIC FIELD X (μT)' in s_df else s_df.filter(like='MAGNETIC FIELD X').iloc[:, 0])
    mag_y = np.interp(uniform_time, s_time, s_df['MAGNETIC FIELD Y (μT)'] if 'MAGNETIC FIELD Y (μT)' in s_df else s_df.filter(like='MAGNETIC FIELD Y').iloc[:, 0])
    mag_z = np.interp(uniform_time, s_time, s_df['MAGNETIC FIELD Z (μT)'] if 'MAGNETIC FIELD Z (μT)' in s_df else s_df.filter(like='MAGNETIC FIELD Z').iloc[:, 0])

    phone_lat = np.interp(uniform_time, s_time, s_df['GPS LATITUDE (degrees)'])
    phone_lon = np.interp(uniform_time, s_time, s_df['GPS LONGITUDE (degrees)'])
    phone_alt = np.interp(uniform_time, s_time, s_df['GPS ALTITUDE (m)'])
    # Convert km/h to m/s
    phone_speed = np.interp(uniform_time, s_time, s_df['GPS SPEED (Kmh)']) / 3.6
    phone_heading = np.interp(uniform_time, s_time, s_df['GPS ORIENTATION (°)'] if 'GPS ORIENTATION (°)' in s_df else s_df.filter(like='GPS ORIENTATION').iloc[:, 0])

    # 5. Ground Truth from V file - FIXED: align with absolute time
    gt_lat = np.interp(uniform_time, v_time, v_df['Latitude (degrees)'])
    gt_lon = np.interp(uniform_time, v_time, v_df['Longitude (degrees)'])
    # Height in V file is in meters (labeled as 'Height (km)' in header due to dataset typo)
    gt_alt = np.interp(uniform_time, v_time, v_df['Height (km)'])
    gt_speed = np.interp(uniform_time, v_time, v_df['Velocity (km/hr)']) / 3.6
    gt_heading = np.interp(uniform_time, v_time, v_df['Heading (degrees)'])

    # 6. Construct unified DataFrame
    synced_df = pd.DataFrame({
        'time': uniform_time,
        'acc_x': acc_x,
        'acc_y': acc_y,
        'acc_z': acc_z,
        'gyro_x': gyro_yaw,    # Mapping based on correlation analysis
        'gyro_y': gyro_roll,   # Mapping based on correlation analysis
        'gyro_z': gyro_pitch,  # Pitch column correlates with yaw rate
        'mag_x': mag_x,
        'mag_y': mag_y,
        'mag_z': mag_z,
        'phone_lat': phone_lat,
        'phone_lon': phone_lon,
        'phone_alt': phone_alt,
        'phone_speed': phone_speed,
        'phone_heading': phone_heading,
        'gt_lat': gt_lat,
        'gt_lon': gt_lon,
        'gt_alt': gt_alt,
        'gt_speed': gt_speed,
        'gt_heading': gt_heading
    })

    return synced_df


def latlon_to_enu(
    lat: np.ndarray,
    lon: np.ndarray,
    alt: np.ndarray,
    lat0: float,
    lon0: float,
    alt0: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Convert geodetic coordinates (lat, lon, alt) to local East-North-Up (ENU) coordinates.
    """
    # WGS84 ellipsoid constants
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = 2 * f - f ** 2

    phi = np.radians(lat)
    lam = np.radians(lon)
    phi0 = np.radians(lat0)
    lam0 = np.radians(lon0)

    # Prime vertical radius of curvature
    def get_N(p):
        return a / np.sqrt(1 - e2 * np.sin(p) ** 2)

    # ECEF coordinates
    N = get_N(phi)
    X = (N + alt) * np.cos(phi) * np.cos(lam)
    Y = (N + alt) * np.cos(phi) * np.sin(lam)
    Z = (N * (1 - e2) + alt) * np.sin(phi)

    N0 = get_N(phi0)
    X0 = (N0 + alt0) * np.cos(phi0) * np.cos(lam0)
    Y0 = (N0 + alt0) * np.cos(phi0) * np.sin(lam0)
    Z0 = (N0 * (1 - e2) + alt0) * np.sin(phi0)

    dX = X - X0
    dY = Y - Y0
    dZ = Z - Z0

    e_x = -np.sin(lam0) * dX - np.cos(lam0) * np.tan(phi0) * dY
    e_y = np.cos(lam0) * dX - np.sin(lam0) * np.tan(phi0) * dY
    e_z = np.tan(phi) * dX + np.cos(phi) * dY

    return e_x, e_y, e_z


def main():
    # Test the fixed synchronization
    s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")
    synced = preprocess_session_fixed(s_df, v_df, target_dt=0.1)

    print(f"Synced dataframe shape: {synced.shape}")
    print(f"First 5 time values: {synced['time'].values[:5]}")
    print(f"Last 5 time values: {synced['time'].values[-5:]}")

if __name__ == "__main__":
    main()