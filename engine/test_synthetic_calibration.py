import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from engine.calibration import CalibrationEngine

def euler_to_rot(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    
    return Rz @ Ry @ Rx

def rot_to_euler(R):
    # Extracts roll, pitch, yaw from rotation matrix
    # Assumes R = Rz * Ry * Rx
    pitch = np.arcsin(-R[2, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])

# 1. Synthetic data setup
np.random.seed(42)
N = 200
dt = 0.1
speed = np.concatenate([np.zeros(100), np.linspace(0, 10, 100)])
acc_veh = np.zeros((N, 3))
acc_veh[:100, 2] = 9.81
acc_veh[100:, 2] = 9.81
acc_veh[100:, 1] = 1.0  # forward accel

# 2. Add realistic MEMS noise (typical smartphone IMU)
# Accel noise: ~0.05 m/s^2 std, Gyro noise: ~0.01 rad/s std, plus some bias
acc_noise = np.random.normal(0, 0.05, (N, 3))
gyro_noise = np.random.normal(0, 0.01, (N, 3)) + np.array([0.002, -0.003, 0.001])

# 3. Apply known rotation
known_roll, known_pitch, known_yaw = 15.0, 10.0, 5.0
R_synth = euler_to_rot(known_roll, known_pitch, known_yaw)

acc_phone = (R_synth @ acc_veh.T).T + acc_noise
gyro_phone = gyro_noise

# 4. Calibration run
calib = CalibrationEngine()
calib.calibrate_from_session(acc_phone, gyro_phone, speed)

print(f"Synthetic Ground Truth Angles: Roll={known_roll:.2f}°, Pitch={known_pitch:.2f}°, Yaw={known_yaw:.2f}°")

# R_synth takes veh -> phone. R_recovered takes phone -> veh.
# R_recovered is approx R_synth.T.
# So R_recovered.T is the recovered veh -> phone transform.
R_recovered_veh_to_phone = calib.R_phone_to_veh.T
rec_roll, rec_pitch, rec_yaw = rot_to_euler(R_recovered_veh_to_phone)
print(f"Recovered Angles (with noise): Roll={rec_roll:.2f}°, Pitch={rec_pitch:.2f}°, Yaw={rec_yaw:.2f}°")

err_roll = abs(rec_roll - known_roll)
err_pitch = abs(rec_pitch - known_pitch)
err_yaw = abs(rec_yaw - known_yaw)
print(f"Angle Errors: Roll Error={err_roll:.2f}°, Pitch Error={err_pitch:.2f}°, Yaw Error={err_yaw:.2f}°")

# 5. Check trigger with noise
R_shifted = euler_to_rot(26.0, 10.0, 5.0) # Roll changed by +11 deg (above 10 deg threshold)
acc_phone_shifted = (R_shifted @ acc_veh.T).T + np.random.normal(0, 0.05, (N, 3))

is_triggered_shifted = calib.check_misalignment_trigger(acc_phone_shifted, speed)
is_triggered_normal = calib.check_misalignment_trigger(acc_phone, speed)

print(f"Trigger on normal noisy data: {is_triggered_normal} (Expected: False)")
print(f"Trigger on shifted (+11°) noisy data: {is_triggered_shifted} (Expected: True)")
