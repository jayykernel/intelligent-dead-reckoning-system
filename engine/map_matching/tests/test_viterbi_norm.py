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

# Monkey-patch match_point to normalize viterbi or avoid cumulative unbounded decrease
orig_match_point = matcher.match_point

def normalized_match_point(raw_pos_enu, heading_deg=None):
    res = orig_match_point(raw_pos_enu, heading_deg)
    if len(matcher.history_states) > 0:
        # Normalize the last viterbi state so max is 0.0
        v_dict = matcher.history_states[-1]["viterbi"]
        max_v = max(v_dict.values())
        for k in v_dict:
            v_dict[k] -= max_v
    return res

matcher.match_point = normalized_match_point

snapped_count = 0
for i in range(200):
    pos = np.array([i * 1.5, 0.5 * np.sin(i*0.1), 0.0]) # 15 m/s
    res = matcher.match_point(pos, 90.0)
    if res.snapped:
        snapped_count += 1

print(f"Normalized Viterbi Snapping: {snapped_count}/200 ({snapped_count/200*100:.1f}%)")
