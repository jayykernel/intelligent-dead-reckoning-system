
import os
import sys
import pandas as pd
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from engine.map_matching.hmm_matcher import HMMMapMatcher

def test_synced_vfa02():
    s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")

    # Calculate exact start time of S in seconds since start of day
    def get_s_sec_of_day(date_str):
        time_part = date_str.split()[1]
        parts = time_part.replace(':', '-').replace('_', '-').split('-')
        h, m, s, ms = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
        return h * 3600 + m * 60 + s + ms / 1000.0

    s_start_sec = get_s_sec_of_day(s_df['DATE (YYYY-MO-DD HH-MI-SS_SSS)'].iloc[0])
    s_time_abs = s_start_sec + (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0
    v_time_abs = v_df['Time Since Start of Day (seconds)'].values

    # Find common time window
    t_start = max(s_time_abs.iloc[0], v_time_abs[0])
    t_end = min(s_time_abs.iloc[-1], v_time_abs[-1])
    dt = 0.1
    uniform_time_abs = np.arange(t_start, t_end, dt)
    uniform_time = uniform_time_abs - t_start

    print(f"Syncing across absolute time: start={t_start:.3f}, end={t_end:.3f}, duration={t_end-t_start:.1f}s")

    acc_x = np.interp(uniform_time_abs, s_time_abs, s_df['ACCELEROMETER X (m/s²)'])
    acc_y = np.interp(uniform_time_abs, s_time_abs, s_df['ACCELEROMETER Y (m/s²)'])
    acc_z = np.interp(uniform_time_abs, s_time_abs, s_df['ACCELEROMETER Z (m/s²)'])

    gyro_yaw = np.interp(uniform_time_abs, s_time_abs, s_df['GYROSCOPE Yaw (rad/s)'])
    gyro_pitch = np.interp(uniform_time_abs, s_time_abs, s_df['GYROSCOPE Pitch (rad/s)'])
    gyro_roll = np.interp(uniform_time_abs, s_time_abs, s_df['GYROSCOPE Roll (rad/s)'])

    gt_lat = np.interp(uniform_time_abs, v_time_abs, v_df['Latitude (degrees)'])
    gt_lon = np.interp(uniform_time_abs, v_time_abs, v_df['Longitude (degrees)'])
    gt_alt = np.interp(uniform_time_abs, v_time_abs, v_df['Height (km)'])
    gt_speed = np.interp(uniform_time_abs, v_time_abs, v_df['Velocity (km/hr)']) / 3.6
    gt_heading = np.interp(uniform_time_abs, v_time_abs, v_df['Heading (degrees)'])

    # Check correlation with GT yaw rate now
    unwrapped_h = np.unwrap(np.radians(gt_heading))
    gt_yaw_rate = np.diff(unwrapped_h) / dt
    gt_yaw_rate = np.append(gt_yaw_rate, gt_yaw_rate[-1])

    for name, g in [('Yaw', gyro_yaw), ('Pitch', gyro_pitch), ('Roll', gyro_roll)]:
        corr = np.corrcoef(g, gt_yaw_rate)[0, 1]
        print(f"Time-aligned Gyro {name} corr with GT yaw rate = {corr:+.4f}")

    # Also check acceleration correlation with derivative of GT speed
    gt_acc = np.diff(gt_speed) / dt
    gt_acc = np.append(gt_acc, gt_acc[-1])
    for name, a in [('AccX', acc_x), ('AccY', acc_y), ('AccZ', acc_z)]:
        corr = np.corrcoef(a, gt_acc)[0, 1]
        print(f"Time-aligned {name} corr with GT forward accel = {corr:+.4f}")

if __name__ == "__main__":
    test_synced_vfa02()
