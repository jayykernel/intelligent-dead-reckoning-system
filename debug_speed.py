import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session, ProductionMobileFusionEngine

# Monkey-patch the step function to capture ai_speed
orig_step = ProductionMobileFusionEngine.step

ai_speeds = []
speed_scales = []

def patched_step(self, *args, **kwargs):
    res = orig_step(self, *args, **kwargs)
    
    # We can retrieve ai_speed...
    # But ai_speed is not in res. We need to grab it from self if we store it.
    
    return res

ProductionMobileFusionEngine.step = patched_step

# Actually, let's just edit `eval/run_full_benchmark.py` locally and read `res['speed_ai']`.
