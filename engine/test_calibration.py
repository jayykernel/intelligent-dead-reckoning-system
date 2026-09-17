import os
import sys
import numpy as np

# Make engine package importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from engine.calibration import CalibrationEngine
from training.data_loader import load_iovnbd_session, preprocess_session

def test_session(driver, session):
    print(f"\n--- Testing Calibration on {driver} / {session} ---")
    s_df, v_df = load_iovnbd_session("data/raw", driver, session)
    synced = preprocess_session(s_df, v_df, target_dt=0.1)
    
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = synced["gt_speed"].values
    
    calib = CalibrationEngine()
    success = calib.calibrate_from_session(acc, gyro, speed)
    if not success:
        print(f"Calibration failed for {session}")
        return
        
    print(f"Calibration Success: {calib.is_calibrated}")
    print(f"Alignment Score: {calib.alignment_score:.2f}")
    print(f"Gyro Bias (rad/s): {calib.gyro_bias}")
    print("Rotation Matrix (Phone -> Vehicle):")
    print(np.round(calib.R_phone_to_veh, 3))
    
    # Test apply
    acc_v, gyro_v = calib.apply(acc, gyro)
    print(f"Mean Vehicle Z-accel (should be ~gravity norm): {np.mean(acc_v[:, 2]):.2f} m/s^2")
    print(f"Mean Vehicle Y-accel (forward): {np.mean(acc_v[:, 1]):.2f} m/s^2")
    print(f"Mean Vehicle X-accel (lateral): {np.mean(acc_v[:, 0]):.2f} m/s^2")
    
if __name__ == "__main__":
    test_session("S (Driver A)", "S4")
    test_session("Vta (Driver E)", "Vta26")
