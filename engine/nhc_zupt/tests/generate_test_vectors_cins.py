import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.nhc_zupt.constrained_ins import ConstrainedINS

def generate():
    cins = ConstrainedINS(dt=0.1)

    test_cases = [
        # 1. Stopped
        {
            "acc_veh": [0.0, 0.0, 9.80665],
            "gyro_veh": [0.01, 0.01, 0.01],
            "v_veh": [0.1, 0.1, 0.0],
            "vehicle_type": "car",
            "lean_angle_rad": 0.0
        },
        # 2. Moving Car
        {
            "acc_veh": [0.2, 1.5, 9.8],
            "gyro_veh": [0.05, 0.02, 0.1],
            "v_veh": [0.5, 12.0, -0.3],
            "vehicle_type": "car",
            "lean_angle_rad": 0.0
        },
        # 3. Moving Two-Wheeler with lean angle
        {
            "acc_veh": [2.5, 0.5, 9.5],
            "gyro_veh": [0.1, 0.05, 0.25],
            "v_veh": [0.8, 14.5, -0.4],
            "vehicle_type": "two_wheeler",
            "lean_angle_rad": 0.25 # ~14.3 degrees
        }
    ]

    out_cases = []
    for tc in test_cases:
        acc = np.array(tc["acc_veh"])
        gyro = np.array(tc["gyro_veh"])
        v = np.array(tc["v_veh"])
        vtype = tc["vehicle_type"]
        lean = tc["lean_angle_rad"]

        is_stopped = cins._is_stopped(acc, gyro, v)
        v_const = cins.constrain(acc, gyro, v, vtype, lean)

        out_cases.append({
            "is_stopped": bool(is_stopped),
            "v_constrained": v_const.tolist()
        })

    os.makedirs("engine/nhc_zupt/tests/vectors", exist_ok=True)
    with open("engine/nhc_zupt/tests/vectors/constrained_ins_in.json", "w") as f:
        json.dump(test_cases, f)
    with open("engine/nhc_zupt/tests/vectors/constrained_ins_out.json", "w") as f:
        json.dump(out_cases, f)

if __name__ == "__main__":
    generate()
