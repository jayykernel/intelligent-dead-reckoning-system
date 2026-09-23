import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from engine.map_matching.hmm_matcher import HMMMapMatcher, RoadNetwork, RoadSegment

rn = RoadNetwork(lat0=0.0, lon0=0.0)
# Break the 1000m road into 20m segments
for i in range(0, 1000, 20):
    rn.segments.append(RoadSegment(i//20, 1001, np.array([float(i), 0.0]), np.array([float(i+20), 0.0])))
rn._build_spatial_index()

matcher = HMMMapMatcher(rn, vehicle_type="car")

# Simulate a 10 Hz path along the road with slight random noise
snapped_count = 0
for i in range(200):
    pos = np.array([i * 1.5, 0.5 * np.sin(i*0.1), 0.0]) # 15 m/s
    res = matcher.match_point(pos, 90.0)
    if res.snapped:
        snapped_count += 1

print(f"Properly discretized road snapping success: {snapped_count}/200 ({snapped_count/200*100:.1f}%)")
