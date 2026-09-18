import numpy as np
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__))))
from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine

s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
synced = preprocess_session(s_df, v_df, target_dt=0.1)
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

print("R_phone_to_veh =")
print(calib.R_phone_to_veh)
