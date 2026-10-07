
import os
import sys
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import (
    load_iovnbd_session,
    preprocess_session,
    latlon_to_enu,
    CalibrationEngine,
    ProductionMobileFusionEngine,
    build_gt_road_network
)

s_df, v_df = load_iovnbd_session("data/raw", "Vf (Driver E)", "V-Vfa02")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)

print("calib.R_phone_to_veh:")
print(calib.R_phone_to_veh)
print("calib.gyro_bias:", calib.gyro_bias)
print("calib.accel_bias:", calib.accel_bias)
print("calib.is_calibrated:", calib.is_calibrated)
print("calib.is_static:", calib.is_static)
