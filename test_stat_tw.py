import numpy as np
from training.data_loader import load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine

synced = load_two_wheeler_session("data/raw/two_wheeler", "session2")
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

acc_veh, gyro_veh = calib.apply(acc, gyro)

# Let's inspect t=45 to 70s (indices 450 to 700)
for i in range(450, 700, 20):
    acc_mag = np.linalg.norm(acc_veh[i])
    gyro_mag = np.linalg.norm(gyro_veh[i])
    print(f"t={i*0.1:.1f}s | Speed: {speed[i]:.2f} m/s | |a|-g: {abs(acc_mag-9.80665):.2f} | |w|: {gyro_mag:.3f} | gyro_z: {gyro_veh[i, 2]:.4f}")
