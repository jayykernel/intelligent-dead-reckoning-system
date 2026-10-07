"""
engine/fusion/tests/diag_nis_cov.py

Phase 1 Diagnostic Script: Analyze NIS Acceptance and EKF Covariance
for GNSS+INS Fusion across a few failing and passing sessions.
"""

import os
import sys
import numpy as np

# Ensure repo root is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import evaluate_dead_reckoning_session

def run_diagnostics():
    sessions = [
        {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"},
        {"category": "car", "driver": "Vf (Driver E)", "session": "V-Vfa02"},
        {"category": "car", "driver": "Vta (Driver E)", "session": "Vta26"},
        {"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"},
        {"category": "car", "driver": "Vw (Driver E)", "session": "Vw16a"},
        {"category": "two_wheeler", "session": "session1"},
    ]

    for sess_config in sessions:
        print(f"\n==========================================")
        print(f"Running Diagnostic for {sess_config['session']}")
        print(f"==========================================")
        try:
            res = evaluate_dead_reckoning_session(sess_config)
            print(f"[{sess_config['session']}] Drift: {res['drift_pct']:.2f}% | NIS Accept: {res['nis_pass_rate']:.2f}%")

            # Print average EKF position and velocity covariance inside and outside outage
            outage_start = res['outage_start']
            outage_end = res['outage_end']
            traj_cov_2d = [r["cov_2d"] for r in res["results"]]

            cov_pre = np.mean(traj_cov_2d[100:outage_start], axis=0)
            cov_out = np.mean(traj_cov_2d[outage_start:outage_end], axis=0)

            print(f"Pre-outage mean pos cov (2D diag): {np.diag(cov_pre)}")
            print(f"Outage mean pos cov (2D diag): {np.diag(cov_out)}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Failed {sess_config['session']}: {e}")

if __name__ == "__main__":
    run_diagnostics()
