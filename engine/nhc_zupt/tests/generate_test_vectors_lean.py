import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.nhc_zupt.lean_ekf import LeanAngleEKF

def generate():
    ekf = LeanAngleEKF(dt=0.1)

    # We will simulate a sequence of inputs
    inputs = [
        # format: gyro_y, acc_x, acc_z, speed, gyro_z, g
        (0.1, 0.5, 9.8, 0.0, 0.0, 9.81), # low speed
        (0.2, 1.0, 9.7, 5.0, 0.3, 9.81), # moving, cornering
        (0.0, 0.1, 9.8, 10.0, 0.01, 9.81), # moving, straight
        (-0.1, -1.0, 9.8, 8.0, -0.2, 9.81) # opposite cornering
    ]

    out_states = []

    for inp in inputs:
        g_y, a_x, a_z, spd, g_z, g = inp
        # Predict
        pred_phi = ekf.predict(gyro_y=g_y)
        # Update
        upd_phi = ekf.update(acc_x=a_x, acc_z=a_z, speed=spd, gyro_z=g_z, g=g)

        out_states.append({
            "pred_phi": float(pred_phi),
            "upd_phi": float(upd_phi),
            "state": ekf.x.tolist(),
            "cov": ekf.P.tolist()
        })

    os.makedirs("engine/nhc_zupt/tests/vectors", exist_ok=True)
    with open("engine/nhc_zupt/tests/vectors/lean_ekf_in.json", "w") as f:
        json.dump(inputs, f)
    with open("engine/nhc_zupt/tests/vectors/lean_ekf_out.json", "w") as f:
        json.dump(out_states, f)

if __name__ == "__main__":
    generate()
