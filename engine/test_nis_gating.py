"""
engine/test_nis_gating.py

Tests Chi-squared NIS gating implementation.
- Verifies valid GNSS updates pass.
- Verifies outliers are rejected.
- Logs NIS stats.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from engine.fusion.ekf import ErrorStateEKF

def test_nis():
    ekf = ErrorStateEKF()
    # Initialize
    ekf.set_initial_state(np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]))

    # Fake measurements
    # 1. Valid update
    z_valid = np.array([0.1, 0.1, 0.0])
    passed_v, nis_v, thresh = ekf.update_gnss_position(z_valid, sigma_pos=0.1)
    print(f"Valid update: passed={passed_v}, NIS={nis_v:.2f}, Threshold={thresh:.2f}")

    # 2. Outlier update (multipath jump)
    z_outlier = np.array([10.0, 10.0, 0.0])
    passed_o, nis_o, thresh_o = ekf.update_gnss_position(z_outlier, sigma_pos=0.1)
    print(f"Outlier update: passed={passed_o}, NIS={nis_o:.2f}, Threshold={thresh_o:.2f}")

    if passed_v and not passed_o:
        print("NIS gating test PASSED.")
    else:
        print("NIS gating test FAILED.")

if __name__ == "__main__":
    test_nis()
