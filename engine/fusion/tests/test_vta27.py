import sys
import numpy as np
import pandas as pd
import math
sys.path.insert(0, r"C:\dev\dead reckoning proto")

from eval.run_full_benchmark import evaluate_dead_reckoning_session

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})
print("\nBenchmark Results for Vta27:")
for k, v in res.items():
    if isinstance(v, float):
        print(f"  {k}: {v:.3f}")
    else:
        print(f"  {k}: {v}")
