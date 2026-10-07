import os
import sys
import numpy as np

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from eval.run_full_benchmark import evaluate_dead_reckoning_session, ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

raw_root = "data/raw"
s_df, v_df = load_iovnbd_session(raw_root, "Vw (Driver E)", "Vw16b")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

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

fusion = ProductionMobileFusionEngine(dt=0.1, default_vehicle_type="car", k=1000.0)
rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
from engine.map_matching.hmm_matcher import HMMMapMatcher
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")
fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

outage_start = 449
outage_end = 1024

for i in range(1, 200):
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])
    m_raw = mag[i] if mag is not None else None

    # Check NIS history length before step
    n_hist_before = len(fusion.ekf.nis_history)
    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=m_raw,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=True,
        timestamp=i * 0.1
    )
    new_updates = fusion.ekf.nis_history[n_hist_before:]
    for u in new_updates:
        if not u["passed"]:
            print(f"Step {i:3d}: Update {u['type']} FAILED NIS: nis={u['nis']:.2f}, thresh={u['threshold']:.2f}")

