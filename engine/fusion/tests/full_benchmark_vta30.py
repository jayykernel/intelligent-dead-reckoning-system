import sys, os, glob
from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.data_loader import preprocess_session, latlon_to_enu, load_iovnbd_session
import numpy as np
import pandas as pd

# Find Vta30 data files
s_files = glob.glob("data/raw/**/Vta30/S-*.csv", recursive=True)
v_files = glob.glob("data/raw/**/Vta30/[Vv]-*.csv", recursive=True)
print(f"S files: {s_files}")
print(f"V files: {v_files}")

# The benchmark expects a session config dict
# Use the correct category and driver for Vta30
session_config = {
    "category": "car",
    "driver": "Vta (Driver E)",
    "session": "Vta30",
}

result = evaluate_dead_reckoning_session(session_config)
print(f"\nResult for Vta30:")
print(f"  Outage dist: {result['outage_dist_m']:.2f} m")
print(f"  Final error: {result['final_error_m']:.2f} m")
print(f"  Drift %: {result['drift_pct']:.2f}%")
print(f"  Official pass: {result['official_pass']}")
print(f"  Outage start: {result['outage_start']}")
print(f"  Outage end: {result['outage_end']}")
