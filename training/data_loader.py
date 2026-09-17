"""
IO-VNBD Data Loader and Preprocessor.

Loads raw S-*.csv (smartphone sensor) and V-*.csv (vehicle ground truth) files,
cleans, synchronizes, normalizes, and packages them into standard numpy/pandas structures.
"""

import os
import glob
import numpy as np
import pandas as pd
from typing import Dict, Tuple, Optional


def load_iovnbd_session(
    data_dir: str,
    driver: str,
    session: str
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """
    Load raw sensor (S) and vehicle (V) CSV files for a given session.
    Handles encoding differences across platforms.
    """
    # Try direct path first (for flexibility in caller)
    session_dir = os.path.join(data_dir, driver, session)
    if os.path.exists(session_dir):
        pass  # Use as-is
    else:
        # Fallback: assume data_dir is the root and prepend "Categorised IOVNB Dataset"
        session_dir = os.path.join(data_dir, "Categorised IOVNB Dataset", driver, session)

    if not os.path.exists(session_dir):
        # Last resort: check if session files are at driver level
        alt_dir = os.path.join(data_dir, "Categorised IOVNB Dataset", driver)
        if os.path.exists(alt_dir) and any(f.endswith('.csv') for f in os.listdir(alt_dir)):
            session_dir = alt_dir
        else:
            raise FileNotFoundError(f"Session directory not found: {session_dir}")

    s_files = glob.glob(os.path.join(session_dir, "S-*.csv")) + glob.glob(os.path.join(session_dir, "s-*.csv"))
    v_files = glob.glob(os.path.join(session_dir, "V-*.csv")) + glob.glob(os.path.join(session_dir, "v-*.csv"))

    if not s_files or not v_files:
        raise FileNotFoundError(f"Missing S or V files in {session_dir}")

    s_df = pd.read_csv(s_files[0], encoding="latin1")
    v_df = pd.read_csv(v_files[0], encoding="latin1")

    # Clean column names (strip whitespace)
    s_df.columns = [c.strip() for c in s_df.columns]
    v_df.columns = [c.strip() for c in v_df.columns]

    return s_df, v_df


def preprocess_session(
    s_df: pd.DataFrame,
    v_df: pd.DataFrame,
    target_dt: float = 0.1
) -> pd.DataFrame:
    """
    Clean, synchronize, and resample raw smartphone and vehicle telemetry.

    Output columns:
    - time: timestamp in seconds starting from 0.0
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
    # 1. Normalize timestamps to seconds relative to start
    # Smartphone time is in ms
    s_time = (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0

    # Vehicle time is in seconds
    v_time = v_df['Time Since Start of Day (seconds)'] - v_df['Time Since Start of Day (seconds)'].iloc[0]

    # Find common duration
    max_duration = min(s_time.iloc[-1], v_time.iloc[-1])
    uniform_time = np.arange(0.0, max_duration, target_dt)

    # 2. Extract and interpolate sensor features
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

    # 3. Ground Truth from V file
    gt_lat = np.interp(uniform_time, v_time, v_df['Latitude (degrees)'])
    gt_lon = np.interp(uniform_time, v_time, v_df['Longitude (degrees)'])
    # Height in V file is in meters (labeled as 'Height (km)' in header due to dataset typo)
    gt_alt = np.interp(uniform_time, v_time, v_df['Height (km)'])
    gt_speed = np.interp(uniform_time, v_time, v_df['Velocity (km/hr)']) / 3.6
    gt_heading = np.interp(uniform_time, v_time, v_df['Heading (degrees)'])

    # 4. Construct unified DataFrame
    synced_df = pd.DataFrame({
        'time': uniform_time,
        'acc_x': acc_x,
        'acc_y': acc_y,
        'acc_z': acc_z,
        'gyro_x': gyro_pitch,   # Angular rate X
        'gyro_y': gyro_roll,    # Angular rate Y
        'gyro_z': gyro_yaw,     # Angular rate Z
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

    east = -np.sin(lam0) * dX + np.cos(lam0) * dY
    north = -np.sin(phi0) * np.cos(lam0) * dX - np.sin(phi0) * np.sin(lam0) * dY + np.cos(phi0) * dZ
    up = np.cos(phi0) * np.cos(lam0) * dX + np.cos(phi0) * np.sin(lam0) * dY + np.sin(phi0) * dZ

    return east, north, up




def load_two_wheeler_session(data_dir: str, session: str, target_dt: float = 0.1) -> pd.DataFrame:
    """
    Load and preprocess a two-wheeler session dataset.
    Interpolates all sensors to target_dt.
    """
    session_dir = os.path.join(data_dir, session)

    acc_df = pd.read_csv(os.path.join(session_dir, "Accelerometer.csv"))
    gyr_df = pd.read_csv(os.path.join(session_dir, "Gyroscope.csv"))
    loc_df = pd.read_csv(os.path.join(session_dir, "Location.csv"))
    mag_df = pd.read_csv(os.path.join(session_dir, "Magnetometer.csv"))

    # Clean headers
    for df in [acc_df, gyr_df, loc_df, mag_df]:
        df.columns = [c.strip().replace('"', '') for c in df.columns]

    time_acc = acc_df['Time (s)'].values
    time_gyr = gyr_df['Time (s)'].values
    time_loc = loc_df['Time (s)'].values

    # We want a common time vector
    t_start = max(time_acc[0], time_gyr[0], time_loc[0])
    t_end = min(time_acc[-1], time_gyr[-1], time_loc[-1])

    uniform_time = np.arange(t_start, t_end, target_dt)

    acc_x = np.interp(uniform_time, time_acc, acc_df.filter(like='Acceleration x').iloc[:, 0])
    acc_y = np.interp(uniform_time, time_acc, acc_df.filter(like='Acceleration y').iloc[:, 0])
    acc_z = np.interp(uniform_time, time_acc, acc_df.filter(like='Acceleration z').iloc[:, 0])

    gyro_x = np.interp(uniform_time, time_gyr, gyr_df.filter(like='Gyroscope x').iloc[:, 0])
    gyro_y = np.interp(uniform_time, time_gyr, gyr_df.filter(like='Gyroscope y').iloc[:, 0])
    gyro_z = np.interp(uniform_time, time_gyr, gyr_df.filter(like='Gyroscope z').iloc[:, 0])

    mag_x = np.interp(uniform_time, mag_df.iloc[:, 0], mag_df.filter(like='Magnetic field x').iloc[:, 0])
    mag_y = np.interp(uniform_time, mag_df.iloc[:, 0], mag_df.filter(like='Magnetic field y').iloc[:, 0])
    mag_z = np.interp(uniform_time, mag_df.iloc[:, 0], mag_df.filter(like='Magnetic field z').iloc[:, 0])

    # Location (ground truth approximation)
    loc_lat = np.interp(uniform_time, time_loc, loc_df.filter(like='Latitude').iloc[:, 0])
    loc_lon = np.interp(uniform_time, time_loc, loc_df.filter(like='Longitude').iloc[:, 0])
    loc_alt = np.interp(uniform_time, time_loc, loc_df.filter(like='Height').iloc[:, 0])

    # Velocity might have NaNs, so ffill/bfill via pandas
    vel_series = pd.Series(loc_df.filter(like='Velocity').iloc[:, 0]).interpolate(method='linear').bfill().ffill()
    loc_vel = np.interp(uniform_time, time_loc, vel_series)

    dir_series = pd.Series(loc_df.filter(like='Direction').iloc[:, 0]).interpolate(method='linear').bfill().ffill()
    loc_heading = np.interp(uniform_time, time_loc, dir_series)

    # Normalize time to start at 0
    uniform_time = uniform_time - uniform_time[0]

    synced_df = pd.DataFrame({
        'time': uniform_time,
        'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z,
        'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z,
        'mag_x': mag_x, 'mag_y': mag_y, 'mag_z': mag_z,
        'phone_lat': loc_lat, 'phone_lon': loc_lon, 'phone_alt': loc_alt,
        'phone_speed': loc_vel, 'phone_heading': loc_heading,
        'gt_lat': loc_lat, 'gt_lon': loc_lon, 'gt_alt': loc_alt,
        'gt_speed': loc_vel, 'gt_heading': loc_heading
    })

    return synced_df
