import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import evaluate_dead_reckoning_session

# Call the standalone Vta26 benchmark config
cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta26"}

res = evaluate_dead_reckoning_session(cfg)

# Analyze drift
outage_dist_m = res["outage_dist_m"]
final_error_m = res["final_error_m"]
drift_pct = res["drift_pct"]

print("==== Vta26 DIAGNOSTICS ====")
print(f"Final Error (m): {final_error_m}")
print(f"Outage Dist (m): {outage_dist_m}")
print(f"Drift Pct (%): {drift_pct}")
print("===========================")

# Let's also dig into the internals
states = res["results"]
speed_scales = [r["speed_scale"] for r in states]
ai_speeds = [r["ai_speed"] for r in states]
modes = [r["mode"] for r in states]

print(f"Speed scale min/max/end: {min(speed_scales)} / {max(speed_scales)} / {speed_scales[-1]}")

drift_ok = abs(drift_pct - (final_error_m / outage_dist_m * 100)) < 0.1 if outage_dist_m > 50 else abs(drift_pct - final_error_m/10) < 0.1
print(f"Drift calc ok? {drift_ok}")

