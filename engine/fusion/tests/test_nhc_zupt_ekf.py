"""
engine/fusion/tests/test_nhc_zupt_ekf.py

Unit test for continuous NHC and ZUPT updates in ErrorStateEKF.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.fusion.ekf import ErrorStateEKF

def test_zupt():
    ekf = ErrorStateEKF(dt=0.1)
    ekf.v = np.array([0.2, -0.1, 0.05])
    passed, nis, thresh = ekf.update_zupt(sigma_zupt=0.05)
    print(f"ZUPT test: passed={passed}, nis={nis:.4f}, thresh={thresh:.4f}")
    print(f"Velocity after ZUPT: {ekf.v}")
    assert passed
    assert np.all(np.abs(ekf.v) < 0.05)
    print("ZUPT test PASSED")

def test_nhc_car():
    ekf = ErrorStateEKF(dt=0.1)
    # Forward along North (Y=10.0 m/s), lateral drift X=0.8 m/s, vertical drift Z=0.3 m/s
    ekf.v = np.array([0.8, 10.0, 0.3])
    passed, nis, thresh = ekf.update_nhc(vehicle_type="car", lean_angle_rad=0.0, sigma_nhc_x=0.2, sigma_nhc_z=0.2)
    print(f"NHC Car test: passed={passed}, nis={nis:.4f}, thresh={thresh:.4f}")
    print(f"Velocity after NHC: {ekf.v}")
    assert passed
    print("NHC Car test PASSED")

if __name__ == "__main__":
    test_zupt()
    test_nhc_car()
