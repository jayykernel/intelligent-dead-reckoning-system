import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from engine.map_matching.hmm_matcher import HMMMapMatcher, RoadNetwork, RoadSegment

rn = RoadNetwork(lat0=0.0, lon0=0.0)
for i in range(0, 1000, 20):
    rn.segments.append(RoadSegment(i//20, 1001, np.array([float(i), 0.0]), np.array([float(i+20), 0.0])))
rn._build_spatial_index()

matcher = HMMMapMatcher(rn, vehicle_type="car")

for i in range(25):
    pos = np.array([i * 1.5, 0.5 * np.sin(i*0.1), 0.0]) # 15 m/s
    res = matcher.match_point(pos, 90.0)
    print(f"[{i:02d}] pos={pos[0]:.1f}, snapped={res.snapped}, fb={res.fallback_reason}")
