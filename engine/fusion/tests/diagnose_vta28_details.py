import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from eval.run_full_benchmark import evaluate_dead_reckoning_session

session_config = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}
res = evaluate_dead_reckoning_session(session_config)
print("Result:", res)
