import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import evaluate_dead_reckoning_session

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta26"}

res = evaluate_dead_reckoning_session(cfg)
outage_start = res["outage_start"]
outage_end = res["outage_end"]

# Just check the results array, I commented out the print inside the library to avoid spam
