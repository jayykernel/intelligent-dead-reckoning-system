import sys, os, numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from training.dataset_splits import TEST_SESSIONS

driver = next(d for d, s in TEST_SESSIONS if s == "Vtb11")
s_df, v_df = load_iovnbd_session("data/raw", driver, "Vtb11")
synced = preprocess_session(s_df, v_df, target_dt=0.1)
N = len(synced)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:min(1200, N)], gyro[:min(1200, N)], speed[:min(1200, N)], dt=0.1)

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
fusion.road_network = build_gt_road_network(e_gt, n_gt, target_segment_length_m=10.0)
from engine.map_matching.hmm_matcher import HMMMapMatcher
fusion.map_matcher = HMMMapMatcher(fusion.road_network, vehicle_type="car")

fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

print("calib.R_phone_to_veh:")
print(calib.R_phone_to_veh)
print("calib.gyro_bias:", calib.gyro_bias)

for i in range(1, 142):
    in_outage = (144 <= i <= 260)
    pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
    h_rad = np.radians(gt_heading[i])
    v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
    
    if i == 141:
        print(f"\n--- STEP {i} DETAILED BREAKDOWN ---")
        acc_raw = acc[i]
        gyro_raw = gyro[i]
        print(f"acc_raw: {acc_raw}, gyro_raw: {gyro_raw}")
        acc_veh, gyro_veh = fusion.calib.apply(acc_raw, gyro_raw)
        print(f"acc_veh: {acc_veh}, gyro_veh: {gyro_veh}")
        
        print(f"Yaw before predict: {fusion.ekf.get_euler_angles_deg()[2]:.2f}")
        ai_speed, sigma_ai, q_scale = fusion.ai_corrector.process_imu_sample(acc_raw=acc_raw, gyro_raw=gyro_raw)
        fusion.ekf.predict(acc_raw=acc_veh, gyro_raw=gyro_veh, dt=fusion.dt, Q_scale=q_scale)
        print(f"Yaw after predict: {fusion.ekf.get_euler_angles_deg()[2]:.2f}")
        
        # NHC
        fusion.ekf.update_nhc(vehicle_type="car", lean_angle_rad=0.0, sigma_nhc_x=0.05, sigma_nhc_z=0.05, alpha=0.01, timestamp=i*0.1)
        print(f"Yaw after NHC: {fusion.ekf.get_euler_angles_deg()[2]:.2f}")
        
        # AI speed
        fusion.ekf.update_ai_forward_speed(speed_fwd=ai_speed, sigma_speed=0.5, alpha=0.01, timestamp=i*0.1)
        print(f"Yaw after AI speed: {fusion.ekf.get_euler_angles_deg()[2]:.2f}")
        
        # Map matching
        # ...
        
        # GNSS pos
        gnss_pos_passed, _, _ = fusion.ekf.update_gnss_position(p_gnss_enu=pos_enu, sigma_pos=5.0, alpha=0.01, timestamp=i*0.1)
        print(f"Yaw after GNSS pos: {fusion.ekf.get_euler_angles_deg()[2]:.2f}")
        
        # GNSS vel
        gnss_vel_passed, _, _ = fusion.ekf.update_gnss_velocity(v_gnss_enu=v_enu, sigma_vel=0.5, alpha=0.01, timestamp=i*0.1)
        print(f"Yaw after GNSS vel: {fusion.ekf.get_euler_angles_deg()[2]:.2f}")
        
        # GNSS heading (at i=141)
        # Note: in step(), GNSS heading is only updated at 1Hz (when int(round(timestamp*10)) % 10 == 0)!
        # At i=141 (14.1s), 141 % 10 == 1 != 0, so GNSS heading is NOT applied!
        print(f"Is 141 % 10 == 0? {141 % 10 == 0}")
        break
        
    res = fusion.step(acc_raw=acc[i], gyro_raw=gyro[i], mag_raw=None, gnss_pos_enu=pos_enu, gnss_vel_enu=v_enu, is_gnss_available=not in_outage, timestamp=i*0.1)
