import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from engine.calibration import CalibrationEngine

# 1. Create Euler to Rotation Matrix helper
def euler_to_rot(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    
    return Rz @ Ry @ Rx

# 2. Synthetic data setup
# Assume pure stationary (gravity) and forward acceleration in vehicle frame
N = 100
# True vehicle Z is [0, 0, 1]
# True vehicle Y is [0, 1, 0]
dt = 0.1
speed = np.concatenate([np.zeros(50), np.linspace(0, 5, 50)])
acc_veh = np.zeros((N, 3))
acc_veh[:50, 2] = 9.81
acc_veh[50:, 2] = 9.81
acc_veh[50:, 1] = 2.0  # constant forward accel

# 3. Apply known rotation
known_roll, known_pitch, known_yaw = 15.0, 10.0, 5.0
R_synth = euler_to_rot(known_roll, known_pitch, known_yaw)
acc_phone = (R_synth @ acc_veh.T).T

gyro_phone = np.zeros((N, 3)) # No rotation in body frame, just alignment

# 4. Calibration run
calib = CalibrationEngine()
calib.calibrate_from_session(acc_phone, gyro_phone, speed)

print(f"Synthetic Injected Roll:{known_roll}, Pitch:{known_pitch}, Yaw:{known_yaw}")
R_recovered = calib.R_phone_to_veh
# Convert recovered matrix back to Euler to compare
# R_synth converts veh to phone (R_synth @ v = p)
# R_recovered is phone to veh
# R_recovered should be approx R_synth.T

diff = R_recovered @ R_synth
print("Rotation Matrix Difference (should be Identity):")
print(np.round(diff, 2))

# 5. Test misalignment trigger
# Shift the phone explicitly by 15 deg in roll
R_shifted = euler_to_rot(30.0, 10.0, 5.0) # Roll changed from 15 to 30
acc_phone_shifted = (R_shifted @ acc_veh.T).T

is_triggered_shifted = calib.check_misalignment_trigger(acc_phone_shifted, speed)
is_triggered_normal = calib.check_misalignment_trigger(acc_phone, speed)

print(f"Trigger on normal (unshifted) data: {is_triggered_normal} (Expected: False)")
print(f"Trigger on shifted (+15 deg) data: {is_triggered_shifted} (Expected: True)")
