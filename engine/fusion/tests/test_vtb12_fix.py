import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath("."))
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.map_matching.hmm_matcher import HMMMapMatcher

class TunedMobileFusionEngine(ProductionMobileFusionEngine):
    def step(self, acc_raw, gyro_raw, mag_raw=None, gnss_pos_enu=None, gnss_vel_enu=None, is_gnss_available=True, timestamp=0.0, **kwargs):
        # Override speed scale learning rate
        ai_speed, sigma_ai, q_scale = self.ai_corrector.process_imu_sample(acc_raw=acc_raw, gyro_raw=gyro_raw)
        
        if is_gnss_available and gnss_vel_enu is not None:
            speed_2d = float(np.linalg.norm(gnss_vel_enu[:2]))
            yaw_rate = abs(gyro_raw[2])
            if ai_speed is not None and speed_2d > 1.5 and ai_speed > 1.0 and yaw_rate < 0.3:
                ratio = speed_2d / ai_speed
                # Fast initial learning rate
                updates = getattr(self, '_speed_scale_updates', 0)
                lr = 0.15 if updates < 30 else (0.05 if updates < 100 else 0.01)
                self._speed_scale_updates = updates + 1
                self.speed_scale = (1.0 - lr) * getattr(self, 'speed_scale', 1.0) + lr * float(np.clip(ratio, 0.1, 4.0))

        return super().step(acc_raw, gyro_raw, mag_raw, gnss_pos_enu, gnss_vel_enu, is_gnss_available, timestamp, **kwargs)

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

fusion = TunedMobileFusionEngine(dt=dt, default_vehicle_type=veh_type, k=1000.0)

rn = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
fusion.road_network = rn
fusion.map_matcher = HMMMapMatcher(rn, vehicle_type=veh_type)

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + int(60.0 / dt)
outage_end = min(outage_end, N - int(10.0 / dt))

results = []
outage_gt_pts = []

for i in range(1, outage_end + 1):
    in_outage = (i >= outage_start)
    if i == outage_start:
        fusion.map_matcher.reset_history()
        fusion._last_matched_point = None
        fusion._last_matched_time = None
        print(f"Entering outage with learned speed_scale={fusion.speed_scale:.3f}")
        
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=None,
        gnss_pos_enu=pos_enu,
        gnss_vel_enu=v_enu,
        is_gnss_available=not in_outage,
        timestamp=i * dt,
        gnss_acc_m=2.0 if not in_outage else None,
        gnss_sat_count=18 if not in_outage else None,
        gnss_avg_cn0=38.0 if not in_outage else None
    )
    results.append(res)
    if in_outage:
        outage_gt_pts.append([e_gt[i], n_gt[i]])

pos_est = np.array([r["pos"] for r in results])
pos_est = np.vstack([p0, pos_est])

outage_gt_pts = np.array(outage_gt_pts)
outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt_pts[:, 0])**2 + np.diff(outage_gt_pts[:, 1])**2)))
final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))
drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0

print(f"Outage Distance: {outage_dist:.2f} m")
print(f"Final Error: {final_err:.2f} m")
print(f"Drift %: {drift_pct:.2f}%")
