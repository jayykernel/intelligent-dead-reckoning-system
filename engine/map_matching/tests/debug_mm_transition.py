import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from engine.map_matching.hmm_matcher import HMMMapMatcher, RoadNetwork, RoadSegment

rn = RoadNetwork(lat0=0.0, lon0=0.0)
rn.segments.append(RoadSegment(1, 1001, np.array([0.0, 0.0]), np.array([1000.0, 0.0])))
rn._build_spatial_index()

matcher = HMMMapMatcher(rn, vehicle_type="car")

# Simulate a 10 Hz path along the road
for i in range(20):
    pos = np.array([i * 1.5, 0.0, 0.0]) # 15 m/s
    res = matcher.match_point(pos, 90.0)
    print(f"[{i}] pos={pos[0]:.1f}, snapped={res.snapped}, fb={res.fallback_reason}")
