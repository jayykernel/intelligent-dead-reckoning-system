import sys, os, glob
from eval.run_full_benchmark import ProductionMobileFusionEngine
from training.data_loader import preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
import numpy as np
import pandas as pd

dt = 0.1
s_files = glob.glob("data/raw/**/Vta30/S-*.csv", recursive=True)
v_files = glob.glob("data/raw/**/Vta30/[Vv]-*.csv", recursive=True)
s_df = pd.read_csv(s_files[0], encoding='latin1')
v_df = pd.read_csv(v_files[0], encoding='latin1')
s_df.columns = [c.strip() for c in s_df.columns]
v_df.columns = [c.strip() for c in v_df.columns]
synced = preprocess_session(s_df, v_df, target_dt=dt)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=100.0)

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

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

outage_start = 3000
outage_end = 3600

ai_speeds_out = []
for i in range(outage_start, outage_end):
    in_outage = True
    pos_enu = None  # no GNSS during outage
    h_rad = np.radians(gt_heading[i])
    v_enu = None

    res = fusion.step(
        acc_raw=acc[i], gyro_raw=gyro[i],
        gnss_pos_enu=pos_enu, gnss_vel_enu=v_enu,
        is_gnss_available=False, timestamp=i*dt,
        gnss_acc_m=None,
    )
    ai_speeds_out.append(res['ai_speed'])
    if i % 300 == 0:
        print(f"t={i*dt:.1f} ai_speed={res['ai_speed']:.2f}")

ai_speeds_out = np.array(ai_speeds_out)
print(f"AI speed during outage: mean={np.mean(ai_speeds_out):.2f}, std={np.std(ai_speeds_out):.2f}, min={np.min(ai_speeds_out):.2f}, max={np.max(ai_speeds_out):.2f}")
print(f"Number of zero AI speeds: {np.sum(ai_speeds_out == 0)}")
print(f"Number of NaN AI speeds: {np.sum(np.isnan(ai_speeds_out))}")
