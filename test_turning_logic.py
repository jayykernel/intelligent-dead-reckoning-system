import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session, ProductionMobileFusionEngine

orig_step = ProductionMobileFusionEngine.step

def new_step(self, *args, **kwargs):
    # Let's inspect
    return orig_step(self, *args, **kwargs)

# Let's run a test by modifying eval/run_full_benchmark.py
