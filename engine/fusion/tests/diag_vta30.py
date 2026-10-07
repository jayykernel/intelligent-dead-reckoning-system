import sys, os, glob
sys.path.insert(0, os.path.abspath("../../.."))
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.map_matching.hmm_matcher import HMMMapMatcher
import numpy as np
import pandas as pd

dt = 0.1
s_files = glob.glob("../../../data/raw/**/Vta30/S-*.csv", recursive=True)
v_files = glob.glob("../../../data/raw/**/Vta30/[Vv]-*.csv", recursive=True)
s_df = pd.read_csv(s_files[0], encoding='latin1')
v_df = pd.read_csv(v_files[0], encoding='latin1')
s_df.columns = [c.strip() for c in s_df.columns]
v_df.columns = [c.strip() for c in v_df.columns]
synced = preprocess_session(s_df, v_df, target_dt=dt)
print(f"Total entries: {len(synced)}")

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
for i in range(1, 3050):
    in_outage = (i >= outage_start)
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

    res = fusion.step(
        acc_raw=acc[i], gyro_raw=gyro[i],
        gnss_pos_enu=pos_enu, gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage, timestamp=i*dt,
        gnss_acc_m=2.0 if not in_outage else None,
    )
    if i % 300 == 0 or i == outage_start:
        print(f"t={i*dt:.1f} scale={getattr(fusion, 'speed_scale', 1.0):.3f} passed_vel={res.get('gnss_vel_passed', True)} ai_speed={res['ai_speed']:.2f} gt={speed[i]:.2f}")
