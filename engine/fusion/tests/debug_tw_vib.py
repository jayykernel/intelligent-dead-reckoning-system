import numpy as np
import os
import sys
from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine

def main():
    raw_path = "data/raw/two_wheeler"
    session_id = 'session1'
    synced = load_two_wheeler_session(raw_path, session_id)

    outage_start = 134.3
    duration = 60.0
    outage_end = outage_start + duration
    dt = 0.1

    acc_raw = synced[['acc_x', 'acc_y', 'acc_z']].values
    gyro_raw = synced[['gyro_x', 'gyro_y', 'gyro_z']].values
    has_mag = 'mag_x' in synced.columns
    mag_raw = synced[['mag_x', 'mag_y', 'mag_z']].values if has_mag else None

    lat_tgt = synced['gt_lat'].values
    lon_tgt = synced['gt_lon'].values
    alt_tgt = synced['gt_alt'].values
    heading_tgt = synced['gt_heading'].values
    speed_tgt = synced['gt_speed'].values
    timestamps = synced['time'].values

    lat0, lon0, alt0 = lat_tgt[0], lon_tgt[0], alt_tgt[0]
    e_gt, n_gt, u_gt = latlon_to_enu(lat_tgt, lon_tgt, alt_tgt, lat0, lon0, alt0)
    pos_gt = np.column_stack((e_gt, n_gt, u_gt))

    vel_gt = np.zeros_like(pos_gt)
    vel_gt[1:] = (pos_gt[1:] - pos_gt[:-1]) / dt
    vel_gt[0] = vel_gt[1]

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc_raw[:1200], gyro_raw[:1200], speed_tgt[:1200], dt=dt)

    fusion = ProductionMobileFusionEngine(dt=dt, k=1000.0, default_vehicle_type="two_wheeler")
    p0 = pos_gt[0]
    v0 = vel_gt[0]
    h0 = heading_tgt[0]

    fusion.initialize_state(
        p0_enu=p0, v0_enu=v0, heading0_deg=h0, acc0_raw=acc_raw[0],
        R_phone_to_veh=calib.R_phone_to_veh, gyro_bias=calib.gyro_bias, accel_bias=calib.accel_bias
    )

    ai_speeds = []

    for i in range(len(timestamps)):
        t = timestamps[i]
        is_gnss_available = not (outage_start <= t < outage_end)

        res = fusion.step(
            acc_raw=acc_raw[i],
            gyro_raw=gyro_raw[i],
            mag_raw=mag_raw[i] if has_mag else None,
            gnss_pos_enu=pos_gt[i] if is_gnss_available else None,
            gnss_vel_enu=vel_gt[i] if is_gnss_available else None,
            is_gnss_available=is_gnss_available,
            timestamp=t
        )

        if not is_gnss_available:
            ai_speeds.append(res['ai_speed'])

    print("Mean AI speed during outage:", np.mean(ai_speeds))
    print("Min AI speed during outage:", np.min(ai_speeds))
    print("Max AI speed during outage:", np.max(ai_speeds))

if __name__ == "__main__":
    main()
