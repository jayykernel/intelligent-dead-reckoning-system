import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import numpy as np
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.sensors.data_types import ImuSample
from core.map.geometry import RoadSegment
from core.map.matcher import MapMatcher
from core.map.multi_hypothesis import MultiHypothesisTracker

def run_ambiguity_test():
    # Straight for 50m, then branches at (+50, 0)
    # Left branch goes North-West, Right branch goes North-East
    seg_trunk = RoadSegment("trunk", (0.0, 0.0, 0.0), (50.0, 0.0, 0.0))
    seg_left = RoadSegment("left", (50.0, 0.0, 0.0), (100.0, -20.0, 0.0))
    seg_right = RoadSegment("right", (50.0, 0.0, 0.0), (100.0, 20.0, 0.0))
    
    # We set a large radius to catch both branches for the demo
    matcher = MapMatcher([seg_trunk, seg_left, seg_right], max_distance_m=30.0)
    mht = MultiHypothesisTracker(matcher, max_hypotheses=3, unconstrained_baseline_prob=0.3)
    
    # Start at origin moving North
    state = NavState(
        timestamp_ns=0, position_m=(0.0, 0.0, 0.0), velocity_mps=(10.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0), accel_bias_mps2=(0.0, 0.0, 0.0), gyro_bias_radps=(0.0, 0.0, 0.0)
    )
    ins = StrapdownINS(state, accel_noise_std=0.01, gyro_noise_std=0.001)
    
    mht.initialize(ins)
    
    dt = 0.1
    time = 0
    ts_ns = 0
    
    print(f"{'Time':<5} | {'Hypotheses (ID:Segment:Score)'}")
    print("-" * 70)
    
    # 1. Move to the branch point (50m North)
    for _ in range(50):
        time += dt
        ts_ns = int(time * 1e9)
        imu = ImuSample(ts_ns, (0.0, 0.0, -9.81), (0.0, 0.0, 0.0))
        mht.propagate(imu)
        
    mht.update_map()
    hyps_str = " | ".join([f"{h.id}:{h.parent_segment_id}:{h.score:.3f}" for h in mht.hypotheses])
    print(f"{time:<5.1f} | {hyps_str}")
    
    # 2. Moving forward another 25m, but drifting slightly right due to unmodeled accel
    # State expects straight, map matcher forces both left and right options!
    for _ in range(25):
        time += dt
        ts_ns = int(time * 1e9)
        # 1m/s^2 accel to the right -> integrates to some drift
        imu = ImuSample(ts_ns, (0.0, 1.0, -9.81), (0.0, 0.0, 0.0))
        mht.propagate(imu)
        
    mht.update_map()
    hyps_str = " | ".join([f"{h.id}:{h.parent_segment_id}:{h.score:.3f}" for h in mht.hypotheses])
    print(f"{time:<5.1f} | {hyps_str}")
    
    # Ensure dominant matches the right branch due to map constraint pulling it gracefully
    dom = mht.get_dominant_hypothesis()
    print(f"\nDom Hyp Post-Ambiguity: {dom.id} on segment {dom.parent_segment_id}")
    
    # 3. Sudden Ground Truth GNSS Fix indicating we actually took the Right branch natively
    obs_pos = (75.0, 10.0, 0.0)
    obs_cov = np.eye(3) * 1.0
    for h in mht.hypotheses:
        # Evaluate map+GNSS update score manually to see pruning effects
        res = h.eskf.update_position(obs_pos, obs_cov)
        score_mod = np.exp(-0.5 * res.mahalanobis_dist**2) if res.accepted else 0.0
        h.score *= score_mod
        
    # Re-normalize
    tot = sum(h.score for h in mht.hypotheses)
    if tot > 0:
        for h in mht.hypotheses: h.score /= tot
    mht.hypotheses.sort(key=lambda x: x.score, reverse=True)
    
    hyps_str = " | ".join([f"{h.id}:{h.parent_segment_id}:{h.score:.4f}" for h in mht.hypotheses])
    print(f"GNSS  | {hyps_str}")
    
    best = mht.get_dominant_hypothesis()
    print(f"Final Resolving Segment: {best.parent_segment_id} (Score {best.score:.4f})")
    
if __name__ == "__main__":
    run_ambiguity_test()
