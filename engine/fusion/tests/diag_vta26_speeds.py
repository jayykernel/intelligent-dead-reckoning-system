import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta26"}

res = evaluate_dead_reckoning_session(cfg)
outage_start = res["outage_start"]
outage_end = res["outage_end"]

print("Final speed scale at outage start:", res['results'][outage_start-1]['speed_scale'])

# Let's count how many times speed scale was actually updated
# We can check by how many unique speed_scale values we have or where it changes
speed_scales = [r['speed_scale'] for r in res['results']]
# Number of changes
changes = sum(1 for i in range(1, len(speed_scales)) if speed_scales[i] != speed_scales[i-1])
print(f"Speed scale changed {changes} times overall.")

# Are there any NaN/overflow?
nan_count = sum(1 for r in res['results'] if np.any(np.isnan(r['pos'])))
print("NaN count:", nan_count)

# Check stability of final error
print("Final Error:", res['final_error_m'])

