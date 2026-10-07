import os
import sys
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import (
    load_iovnbd_session,
    preprocess_session,
    latlon_to_enu,
    CalibrationEngine,
    ProductionMobileFusionEngine,
    build_gt_road_network
)

def diag_vfa02():
    raw_root = "data/raw"
    driver = "Vf (Driver E)"
    session_name = "V-Vfa02"
    dt = 0.1

    s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
    synced = preprocess_session(s_df, v_df, target_dt=dt)
    N = len(synced)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    
    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

    p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
    heading_rad = np.radians(gt_heading[0])
    v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

    fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=1000.0)
    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    # Watch speed_scale evolution
    print("Step, AI Speed, GT Speed, Speed Scale")
    
    for i in range(1, 1000): # Just first 1000 steps
        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            gnss_pos_enu=np.array([e_gt[i], n_gt[i], u_gt[i]]),
            gnss_vel_enu=np.array([speed[i] * np.sin(np.radians(gt_heading[i])), speed[i] * np.cos(np.radians(gt_heading[i])), 0.0]),
            is_gnss_available=True,
            timestamp=i * dt
        )
        
        # Access speed_scale from fusion engine
        # In engine/fusion/fusion_engine.py / eval/run_full_benchmark.py
        # ProductionMobileFusionEngine has self.speed_scale
        
        if i % 50 == 0:
            print(f"{i}, {res['ai_speed']:.2f}, {speed[i]:.2f}, {res['speed_scale']:.4f}")

if __name__ == "__main__":
    diag_vfa02()
