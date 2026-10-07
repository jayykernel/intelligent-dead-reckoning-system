import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.fusion.tests.test_vw16a_experiments import run_sim

print("4. Baseline, no map matching:", run_sim(disable_map_matching=True))
print("5. Fixed scale 1.0 + no map matching:", run_sim(fixed_speed_scale=1.0, disable_map_matching=True))
