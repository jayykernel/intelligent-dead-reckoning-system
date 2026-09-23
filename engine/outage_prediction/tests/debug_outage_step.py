import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from eval.test_closed_loop_map_matching import run_closed_loop_test

# Let's inspect the first 20 steps of the outage
