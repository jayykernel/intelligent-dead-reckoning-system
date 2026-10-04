import numpy as np
import os
from training.data_loader import load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine
from training.data_loader import latlon_to_enu

# Load session 2
raw_path = os.path.join("data/raw/two_wheeler")
synced = load_two_wheeler_session(raw_path, 'session2')
dt = 0.1
acc = synced[['acc_x', 'acc_y', 'acc_z']].values
gyro = synced[['gyro_x', 'gyro_y', 'gyro_z']].values
speed = np.nan_to_num(synced['gt_speed'].values, nan=0.0)
mag = synced[['mag_x', 'mag_y', 'mag_z']].values if 'mag_x' in synced.columns else None

# Calibrate
calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

# Setup
fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type='two_wheeler', k=1000.0)
# Initialize state...
lat0 = synced['gt_lat'].iloc[0]
lon0 = synced['gt_lon'].iloc[0]
alt0 = synced['gt_alt'].iloc[0]
e_gt, n_gt, u_gt = latlon_to_enu(synced['gt_lat'].values, synced['gt_lon'].values, synced['gt_alt'].values, lat0, lon0, alt0)
p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
gt_heading = synced['gt_heading'].values
# ... (simplified init)
fusion.initialize_state(p0, np.zeros(3), gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

print("Time(s) | AI_Raw | AI_Scaled | Speed_Scale | GT_Speed | Yaw_Rate")
for i in range(1, 1000):
    res = fusion.step(
        acc_raw=acc[i],
        gyro_raw=gyro[i],
        mag_raw=mag[i] if mag is not None else None,
        gnss_pos_enu=np.array([e_gt[i], n_gt[i], u_gt[i]]),
        gnss_vel_enu=np.array([0, speed[i], 0]), # Approximation
        is_gnss_available=True,
        timestamp=i * dt
    )
    if i % 10 == 0:
        raw_ai = res.get("ai_raw", 0.0) # Need to check if this key exists or if I need to adjust step
        # Ah, step doesn't return "ai_raw" - I need to inspect the EKF state or ai_filter internally.
        # Let's just log what we have.
        print(f"{i*dt:.1f} | S:{res['speed_scale']:.3f} | AI:{res['ai_speed']:.2f} | GT:{speed[i]:.2f}")
