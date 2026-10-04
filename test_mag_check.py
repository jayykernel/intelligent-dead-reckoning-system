import numpy as np
from training.data_loader import load_two_wheeler_session
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.mag_gate import MagnetometerGate

synced = load_two_wheeler_session("data/raw/two_wheeler", "session2")
mag = synced[["mag_x", "mag_y", "mag_z"]].values
acc = synced[["acc_x", "acc_y", "acc_z"]].values
gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

calib = CalibrationEngine()
calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)
calib.calibrate_magnetometer(mag[:1200])

gate = MagnetometerGate()
print("Without mag calib:")
for i in range(0, 100, 20):
    mag_veh = mag[i] @ calib.R_phone_to_veh.T
    clean, yaw, debug = gate.process_measurement(mag_veh, np.eye(3))
    print(f"  t={i*0.1:.1f}s | Clean: {clean} | Mag norm: {debug['mag_norm']:.1f} | Reason: {debug['reason']}")

print("\nWith mag calib (calib.apply_mag_calibration):")
gate2 = MagnetometerGate()
for i in range(0, 100, 20):
    mag_cal = calib.apply_mag_calibration(mag[i])
    mag_veh = mag_cal @ calib.R_phone_to_veh.T
    clean, yaw, debug = gate2.process_measurement(mag_veh, np.eye(3))
    print(f"  t={i*0.1:.1f}s | Clean: {clean} | Mag norm: {debug['mag_norm']:.1f} | Reason: {debug['reason']}")
