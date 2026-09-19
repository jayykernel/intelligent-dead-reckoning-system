
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.fusion_engine import GNSSINSFusionEngine

# Load TW session 1
raw_root = "data/raw"
synced = load_two_wheeler_session(os.path.join(raw_root, "two_wheeler"), "session1")
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = synced["gt_speed"].values

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

# Inits
fusion = GNSSINSFusionEngine(dt=0.1, enable_ai=False)
fusion.initialize_state(np.zeros(3), np.zeros(3), 0.0, acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

print("Step | b_az | Acc_veh_z")
for i in range(1, 1000):
    acc_veh, gyro_veh = fusion.calib.apply(acc[i], gyro[i])
    fusion.ekf.predict(acc_veh, gyro_veh, dt=0.1)

    if i % 100 == 0:
        print(f"{i:04d} | {fusion.ekf.b_a[2]:.4f} | {acc_veh[2]:.4f}")
