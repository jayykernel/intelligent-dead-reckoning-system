import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath("."))
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.map_matching.hmm_matcher import HMMMapMatcher

driver = "Vtb (Driver E)"
session_name = "Vtb12"
raw_root = "data/raw"
dt = 0.1

s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)
synced = preprocess_session(s_df, v_df, target_dt=dt)
veh_type = "car"

N = len(synced)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
if mag is not None:
    calib.calibrate_magnetometer(mag[:1200])

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

fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type=veh_type, k=1000.0)

rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type=veh_type)

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / dt)
outage_end = min(outage_end, N - int(10.0 / dt))
eval_end = min(N, outage_end + int(10.0 / dt))

logs = []
for i in range(1, eval_end):
    in_outage = (outage_start <= i <= outage_end)
    if i == outage_start:
        fusion.map_matcher.reset_history()
        fusion._last_matched_point = None
        fusion._last_matched_time = None

    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
    m_raw = mag[i] if mag is not None else None

    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=m_raw,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage,
        timestamp=i * dt,
        gnss_acc_m=2.0 if not in_outage else None,
        gnss_sat_count=18 if not in_outage else None,
        gnss_avg_cn0=38.0 if not in_outage else None
    )

    yaw = res["euler_deg"][2]

    logs.append({
        "t": i*dt,
        "gnss_avail": not in_outage,
        "gt_x": e_gt[i],
        "gt_y": n_gt[i],
        "gt_speed": speed[i],
        "est_x": res["pos"][0],
        "est_y": res["pos"][1],
        "est_speed": np.linalg.norm(res["vel"][:2]),
        "ai_speed": res["ai_speed"],
        "speed_scale": res["speed_scale"],
        "yaw_deg": yaw,
        "gt_yaw": gt_heading[i],
        "is_stopped": (i*dt > 0 and fusion.constrained_ins._is_stopped(
           np.array([0,0,9.8]), np.array([0,0,0]), res["vel"], ai_speed=res["ai_speed"], vehicle_type="car")), # approximate
    })

df = pd.DataFrame(logs)
outage_df = df[~df["gnss_avail"]]
if len(outage_df) > 0:
    err_end = np.linalg.norm(np.array([outage_df.iloc[-1]["est_x"] - outage_df.iloc[-1]["gt_x"],
                                      outage_df.iloc[-1]["est_y"] - outage_df.iloc[-1]["gt_y"]]))
    dist_num = np.sum(np.sqrt(np.diff(outage_df["gt_x"])**2 + np.diff(outage_df["gt_y"])**2))
    print(f"Drift at end: {err_end:.2f}m over {dist_num:.2f}m = {err_end/dist_num*100:.2f}%")
    print(f"Mean AI speed: {outage_df['ai_speed'].mean():.2f}")
    print(f"Mean GT speed: {outage_df['gt_speed'].mean():.2f}")
    print(f"Mean Speed Scale: {outage_df['speed_scale'].mean():.3f}")

# Look at speed scale behavior before outage
pre_outage = df[df["gnss_avail"] & (df["t"] < df[~df["gnss_avail"]]["t"].min())]
print("\nPre-outage stats:")
print(f"Final scale: {pre_outage.iloc[-1]['speed_scale']:.3f}")
print(f"Mean AI speed: {pre_outage['ai_speed'].mean():.2f}")
print(f"Mean GT speed: {pre_outage['gt_speed'].mean():.2f}")
print(f"AI > 1.0 count: {(pre_outage['ai_speed'] > 1.0).sum()}")

# Map MATCHING evaluation
print("\nDuring Outage Map Matching Stats:")
mm_count = 0
for t, x, y, gyaw in zip(outage_df["t"], outage_df["est_x"], outage_df["est_y"], outage_df["gt_yaw"]):
    pass # we can add if needed
