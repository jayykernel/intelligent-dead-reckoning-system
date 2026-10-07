import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import numpy as np
import pandas as pd
from eval.run_full_benchmark import load_iovnbd_session, preprocess_session, ProductionMobileFusionEngine
from engine.calibration.calibrator import CalibrationEngine

def analyze_vw15():
    raw_root = "data/raw"
    driver = "Vw (Driver E)"
    session_name = "Vw15"
    dt = 0.1

    print(f"Loading {session_name}...")
    s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
    synced = preprocess_session(s_df, v_df, target_dt=dt)

    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

    p0 = np.array([0.0, 0.0, 0.0])
    heading_rad = np.radians(gt_heading[0])
    v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

    fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=1000.0)
    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    outage_start = min(int(N * 0.4), 3000)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))

    gnss_vels = []
    ai_speeds = []
    speed_scales = []
    has_nan = False
    cov_start = None
    cov_end = None

    for i in range(1, outage_end + 100):
        in_outage = (outage_start <= i <= outage_end)

        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=None,
            gnss_pos_enu=np.array([0.0, 0.0, 0.0]) if not in_outage else None, # fake pos for structural purpose if needed, but only looking at speed logic
            gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage,
            timestamp=i * dt
        )

        if np.any(np.isnan(res["pos"])) or np.any(np.isnan(res["vel"])):
            has_nan = True

        if not in_outage and i > max(1, outage_start - 100):
            gnss_vels.append(np.linalg.norm(v_enu[:2]) if v_enu is not None else 0.0)
            ai_speeds.append(res["ai_speed"])

        if i == outage_start - 1:
            print(f"Pre-outage (t={i*dt}s):")
            print(f"  GNSS Velocity: {np.linalg.norm(v_enu[:2]):.3f} m/s")
            print(f"  AI Speed: {res['ai_speed']:.3f} m/s")
            print(f"  Speed Scale: {res['speed_scale']:.3f}")
            print(f"  Final state vel: {np.linalg.norm(res['vel'][:2]):.3f} m/s")

        if i == outage_start:
            cov_start = res['cov_2d']
        if i == outage_end:
            cov_end = res['cov_2d']

        speed_scales.append(res['speed_scale'])

    print(f"Covariance Start: {cov_start}")
    print(f"Covariance End: {cov_end}")

    ai_speed_mean = np.mean(ai_speeds)
    gnss_vel_mean = np.mean(gnss_vels)

    print("\nOverall Stats:")
    print(f"Has NaN: {has_nan}")
    print(f"Avg AI Speed before outage: {ai_speed_mean:.3f}")
    print(f"Avg GNSS Vel before outage: {gnss_vel_mean:.3f}")
    print(f"Initial Speed Scale: {speed_scales[0]:.3f}")
    print(f"Final Speed Scale: {speed_scales[-1]:.3f}")
    print(f"Speed scale is learning? {speed_scales[0] != speed_scales[-1] and abs(speed_scales[-1] - 1.0) > 1e-4}")

if __name__ == "__main__":
    analyze_vw15()