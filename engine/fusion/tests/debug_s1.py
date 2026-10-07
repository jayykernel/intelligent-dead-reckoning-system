import numpy as np
import os
import json
import logging
from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
import sys

# Need to load the eval framework's fusion engine structure
from eval.run_full_benchmark import ProductionMobileFusionEngine

def main():
    raw_path = "data/raw/two_wheeler"
    if not os.path.exists(raw_path):
        return

    session_id = 'session1'
    try:
        synced = load_two_wheeler_session(raw_path, session_id)
    except Exception as e:
        print(f"Failed to load {session_id}: {e}")
        return

    # Load Outage definition for session1 from run_full_benchmark.py
    # In run_full_benchmark:
    # "session1": {"outage_start": 134.3, "duration": 60.0},

    outage_start = 134.3
    duration = 60.0
    outage_end = outage_start + duration

    dt = 0.1
    # Extract data
    acc_raw = synced[['acc_x', 'acc_y', 'acc_z']].values
    gyro_raw = synced[['gyro_x', 'gyro_y', 'gyro_z']].values

    has_mag = 'mag_x' in synced.columns
    if has_mag:
        mag_raw = synced[['mag_x', 'mag_y', 'mag_z']].values
    else:
        mag_raw = np.zeros_like(acc_raw)

    lat_tgt = synced['gt_lat'].values
    lon_tgt = synced['gt_lon'].values
    alt_tgt = synced['gt_alt'].values
    heading_tgt = synced['gt_heading'].values
    speed_tgt = synced['gt_speed'].values
    timestamps = synced['time'].values

    lat0, lon0, alt0 = lat_tgt[0], lon_tgt[0], alt_tgt[0]
    e_gt, n_gt, u_gt = latlon_to_enu(lat_tgt, lon_tgt, alt_tgt, lat0, lon0, alt0)
    pos_gt = np.column_stack((e_gt, n_gt, u_gt))

    # GNSS velocity pseudo-truth
    vel_gt = np.zeros_like(pos_gt)
    vel_gt[1:] = (pos_gt[1:] - pos_gt[:-1]) / dt
    vel_gt[0] = vel_gt[1]

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc_raw[:1200], gyro_raw[:1200], speed_tgt[:1200], dt=dt)

    fusion = ProductionMobileFusionEngine(dt=dt, k=100.0, default_vehicle_type="two_wheeler")
    p0 = pos_gt[0]
    v0 = vel_gt[0]
    h0 = heading_tgt[0]

    fusion.initialize_state(
        p0_enu=p0, v0_enu=v0, heading0_deg=h0, acc0_raw=acc_raw[0],
        R_phone_to_veh=calib.R_phone_to_veh, gyro_bias=calib.gyro_bias, accel_bias=calib.accel_bias
    )

    print("Time | isGNSS | Speed2D | AI | ratio | S_Scale | ZUPT | fwd_spd")
    for i in range(len(timestamps)):
        t = timestamps[i]
        is_gnss_available = not (outage_start <= t < outage_end)

        gnss_pos = pos_gt[i] if is_gnss_available else None
        gnss_vel = vel_gt[i] if is_gnss_available else None

        # We need to capture AI speed and speed scale inside step. We can patch or just print after.
        res = fusion.step(
            acc_raw=acc_raw[i],
            gyro_raw=gyro_raw[i],
            mag_raw=mag_raw[i] if has_mag else None,
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            is_gnss_available=is_gnss_available,
            timestamp=t
        )

        speed_2d = float(np.linalg.norm(vel_gt[i][:2]))

        if (outage_start - 1.0 < t < outage_start + 5.0) or t % 10.0 < 0.1:
            print(f"{t:5.1f} | {is_gnss_available} | {speed_2d:7.2f} | {res['ai_speed'] if res['ai_speed'] is not None else 0:5.2f} | " +
                  f" {(speed_2d/(res['ai_speed'] if res['ai_speed'] else 1.0)):5.2f} | {getattr(fusion, 'speed_scale', 1.0):.3f} | {res.get('stopped', False)} | {np.linalg.norm(fusion.ekf.v[:2]):5.2f}")

if __name__ == "__main__":
    main()
