import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from eval.test_closed_loop_map_matching import build_gt_road_network
from engine.map_matching.hmm_matcher import HMMMapMatcher

e_gt = np.linspace(0, 1000, 100)
n_gt = np.zeros(100)
rn = build_gt_road_network(e_gt, n_gt)
matcher = HMMMapMatcher(rn, vehicle_type="car")

print("Number of segments in network:", len(rn.segments))

res = matcher.match_point(np.array([10.0, 2.0, 0.0]), 90.0)
print("Point match result:", res.snapped, res.fallback_reason)
