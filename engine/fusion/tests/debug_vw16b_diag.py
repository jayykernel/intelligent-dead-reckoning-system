import os
import sys
import numpy as np
import pandas as pd

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from eval.run_full_benchmark import evaluate_dead_reckoning_session

config = {
    "category": "car",
    "driver": "Vw (Driver E)",
    "session": "Vw16b"
}

try:
    res = evaluate_dead_reckoning_session(config)
    print("Result:", res)
except Exception as e:
    import traceback
    traceback.print_exc()
