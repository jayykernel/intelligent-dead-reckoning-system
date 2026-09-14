import pytest
import numpy as np
from core.map.geometry import RoadSegment
from core.map.matcher import MapMatcher
from core.map.multi_hypothesis import MultiHypothesisTracker
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.sensors.data_types import ImuSample

def create_ins(x, y, vx, vy):
    state = NavState(
        timestamp_ns=0, position_m=(x, y, 0.0), velocity_mps=(vx, vy, 0.0),
        attitude_q_v2n=(1.0,0.0,0.0,0.0), accel_bias_mps2=(0.0,0.0,0.0), gyro_bias_radps=(0.0,0.0,0.0)
    )
    ins = StrapdownINS(state)
    ins.covariance = np.eye(15) * 1.0 # High uncertainty
    return ins

def test_single_unambiguous_road():
    seg = RoadSegment("main_road", (0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    matcher = MapMatcher([seg])
    mht = MultiHypothesisTracker(matcher)
    
    # Very close to main road
    ins = create_ins(50.0, 1.0, 10.0, 0.0)
    mht.initialize(ins)
    mht.update_map()
    
    # Should have two hypotheses: 1 mapped, 1 unconstrained baseline
    assert len(mht.hypotheses) == 2
    dom = mht.get_dominant_hypothesis()
    assert dom.parent_segment_id == "main_road"

def test_two_plausible_roads():
    # Y shape branch
    seg1 = RoadSegment("left_branch", (0.0, 0.0, 0.0), (100.0, -10.0, 0.0))
    seg2 = RoadSegment("right_branch", (0.0, 0.0, 0.0), (100.0, 10.0, 0.0))
    matcher = MapMatcher([seg1, seg2], max_distance_m=15.0)
    mht = MultiHypothesisTracker(matcher, max_hypotheses=3)
    
    # State is between both branches (y=0) at x=50
    # Left is at y=-5, Right is at y=5. Both are reachable.
    ins = create_ins(50.0, 0.0, 10.0, 0.0)
    mht.initialize(ins)
    mht.update_map()
    
    # Max hypotheses is 3. We have baseline, left, right.
    assert len(mht.hypotheses) == 3
    ids = [h.parent_segment_id for h in mht.hypotheses]
    assert "left_branch" in ids
    assert "right_branch" in ids
    assert None in ids # the baseline
    
def test_incorrect_road_rejection():
    seg = RoadSegment("distant_road", (0.0, 100.0, 0.0), (100.0, 100.0, 0.0))
    matcher = MapMatcher([seg])
    mht = MultiHypothesisTracker(matcher)
    
    # Far from road
    ins = create_ins(50.0, 0.0, 10.0, 0.0)
    mht.initialize(ins)
    mht.update_map()
    
    # Only baseline should survive because distance > 15m rejects the constrained branch
    assert len(mht.hypotheses) == 1
    assert mht.hypotheses[0].parent_segment_id is None
    
def test_baseline_preservation():
    seg = RoadSegment("road", (0.0, 10.0, 0.0), (100.0, 10.0, 0.0))
    matcher = MapMatcher([seg])
    # Give baseline a higher probability
    mht = MultiHypothesisTracker(matcher, unconstrained_baseline_prob=1.5)
    
    ins = create_ins(50.0, 0.0, 10.0, 0.0)
    # The road is a match (10m away), but the baseline prob is high
    mht.initialize(ins)
    mht.update_map()
    
    # Baseline should win due to high prob
    dom = mht.get_dominant_hypothesis()
    assert dom.parent_segment_id is None
