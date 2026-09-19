import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.outage_prediction.outage_predictor import OutagePredictor

def generate():
    predictor = OutagePredictor(window_size_sec=3.0, dt=1.0)

    # Sequence of GNSS states representing an approaching tunnel
    inputs = [
        {"cn0": 38.0, "sat": 14, "acc": 2.0},  # Strong signal
        {"cn0": 36.0, "sat": 13, "acc": 2.5},
        {"cn0": 28.0, "sat": 10, "acc": 4.0},  # Sudden drop
        {"cn0": 22.0, "sat": 6, "acc": 8.0},   # Approaching failure
        {"cn0": 18.0, "sat": 3, "acc": 25.0},  # Below threshold
        {"cn0": 18.0, "sat": 3, "acc": 25.0},  # Prolonged outage
        {"cn0": 33.0, "sat": 9, "acc": 5.0},   # Sudden recovery
        {"cn0": 37.0, "sat": 12, "acc": 3.0}   # Full recovery
    ]

    out_states = []

    for inp in inputs:
        trust = predictor.update(avg_cn0=inp["cn0"], sat_count=inp["sat"], accuracy_m=inp["acc"])

        out_states.append({
            "trust": float(trust)
        })

    os.makedirs("engine/outage_prediction/tests/vectors", exist_ok=True)
    with open("engine/outage_prediction/tests/vectors/outage_predictor_in.json", "w") as f:
        json.dump(inputs, f)
    with open("engine/outage_prediction/tests/vectors/outage_predictor_out.json", "w") as f:
        json.dump(out_states, f)

if __name__ == "__main__":
    generate()
