import sys
import os
import numpy as np
import pandas as pd

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from eval.run_full_benchmark import (
    load_iovnbd_session,
    load_two_wheeler_session,
    preprocess_session,
    latlon_to_enu,
    ProductionMobileFusionEngine
)
from engine.calibration.calibrator import CalibrationEngine

def run_feasibility(session_config):
    category = session_config["category"]
    driver = session_config.get("driver")
    session_name = session_config["session"]
    raw_root = "data/raw"
    dt = 0.1

    print(f"\n--- Checking Map-Matching Feasibility on {category.upper()}: {session_name} ---")

    if category == "car":
        s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
        synced = preprocess_session(s_df, v_df, target_dt=dt)
        veh_type = "car"
    elif category == "two_wheeler":
        synced = load_two_wheeler_session(os.path.join(raw_root, "two_wheeler"), session_name)
        veh_type = "two_wheeler"

    N = len(synced)
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:min(1200, N)], gyro[:min(1200, N)], speed[:min(1200, N)], dt=dt)

    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)

    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        vx = np.gradient(e_gt, dt)
        vy = np.gradient(n_gt, dt)
        gt_heading = np.degrees(np.arctan2(vx, vy)) % 360.0

    vx_gt = np.gradient(e_gt, dt)
    vy_gt = np.gradient(n_gt, dt)
    vz_gt = np.gradient(u_gt, dt)

    # Pick outage
    moving_mask = speed > 3.0
    moving_indices = np.where(moving_mask)[0]
    outage_len = int(60.0 / dt)
    
    start_idx = moving_indices[len(moving_indices) // 4]
    end_idx = start_idx + outage_len

    engine = ProductionMobileFusionEngine(k=100.0, dt=dt)
    engine.calib = calib
    engine.current_vehicle_type = veh_type

    p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
    v0 = np.array([vx_gt[0], vy_gt[0], vz_gt[0]])
    h0 = gt_heading[0]
    engine.initialize_state(p0, v0, h0, acc[0])

    dr_pos = []
    gt_pos = []

    for i in range(min(N, end_idx + 50)):
        in_outage = (start_idx <= i <= end_idx)
        
        gps_p = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        gps_v = np.array([vx_gt[i], vy_gt[i], vz_gt[i]]) if not in_outage else None
        
        res = engine.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            gnss_pos_enu=gps_p,
            gnss_vel_enu=gps_v,
            is_gnss_available=not in_outage,
            timestamp=i * dt,
            gnss_acc_m=2.0 if not in_outage else 99.0,
            gnss_sat_count=16 if not in_outage else 0,
            gnss_avg_cn0=38.0 if not in_outage else 0.0
        )

        if in_outage:
            dr_pos.append(res['pos'][:2])
            gt_pos.append(np.array([e_gt[i], n_gt[i]]))

    dr_pos = np.array(dr_pos)
    gt_pos = np.array(gt_pos)

    distances = np.linalg.norm(dr_pos - gt_pos, axis=1)
    max_radius = 25.0 if veh_type == "car" else 45.0
    
    snapped = distances <= max_radius
    snap_pct = np.mean(snapped) * 100

    print(f"Outage duration: 60.0s ({start_idx*dt:.1f}s to {end_idx*dt:.1f}s)")
    print(f"Max distance from GT road centerline: {np.max(distances):.2f} m")
    print(f"Mean distance: {np.mean(distances):.2f} m")
    print(f"Percentage of outage within Map Matching radius ({max_radius}m): {snap_pct:.1f}%")

    first_loss = np.where(~snapped)[0]
    if len(first_loss) > 0:
        print(f"Map Matching would FALL BACK (lose snap) at T + {first_loss[0]*dt:.1f}s into outage.")
    else:
        print("Map Matching would STAY SNAPPED for the entire 60s outage!")

if __name__ == "__main__":
    run_feasibility({"category": "car", "driver": "S (Driver A)", "session": "S4"})
    run_feasibility({"category": "two_wheeler", "session": "session1"})
