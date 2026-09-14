import pytest
import numpy as np
from core.map.geometry import RoadSegment
from core.map.matcher import MapMatcher
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.map.multi_hypothesis import MultiHypothesisTracker

def create_base_ins(z_pos=0.0):
    return StrapdownINS(
        NavState(
            timestamp_ns=0,
            position_m=(0.0, 0.0, z_pos),
            velocity_mps=(10.0, 0.0, 0.0), # moving north
            attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
            accel_bias_mps2=(0.0, 0.0, 0.0),
            gyro_bias_radps=(0.0, 0.0, 0.0)
        )
    )

def test_missing_elevation_evidence():
    # Road without elevation vs road with elevation
    seg_no_elevation = RoadSegment("flat", (0, 0, 0), (100, 0, 0), has_elevation=False)
    
    matcher = MapMatcher([seg_no_elevation], cross_track_variance=1.0, vertical_variance=1.0)
    
    # Vehicle is at z = -50 (50 meters up!)
    ins = create_base_ins(z_pos=-50.0)
    
    res = matcher.match_all(ins.state)
    assert len(res) == 1
    # Because has_elevation=False, vertical_variance is overridden to 1e9
    # The projected Z should equal the state Z, meaning 0 innovation.
    assert res[0].projected_pos_ned[2] == -50.0
    assert res[0].observation_cov[2, 2] == 1e9

def test_same_horizontal_different_levels():
    # Two perfectly overlapping roads in 2D, differing only in Z
    # Ground road at z = 0
    seg_ground = RoadSegment("ground", (0, 0, 0.0), (100, 0, 0.0), has_elevation=True)
    # Flyover at z = -10 (NED down is positive, so -10 is up)
    seg_flyover = RoadSegment("flyover", (0, 0, -10.0), (100, 0, -10.0), has_elevation=True)
    
    matcher = MapMatcher([seg_ground, seg_flyover], cross_track_variance=1.0, vertical_variance=4.0)
    
    # Vehicle is driving on the flyover (z = -9.0)
    ins = create_base_ins(z_pos=-9.0)
    # Add some uncertainty to altitude so both could theoretically be evaluated
    ins.covariance[2, 2] = 9.0
    
    mht = MultiHypothesisTracker(matcher, max_hypotheses=3)
    mht.initialize(ins)
    mht.update_map()
    
    dom = mht.get_dominant_hypothesis()
    # Should confidently pick flyover and reject or down-rank ground
    assert dom.parent_segment_id == "flyover"
    
    # Ground should be rejected because 9m error with 4m variance gives high Mahalanobis distance
    # innovation = 9.0. M = 9 / sqrt(4 + 9) = 9 / 3.6 = 2.5. If gate=4, it passes, but score is much lower.
    flyover_hyp = next((h for h in mht.hypotheses if h.parent_segment_id == "flyover"), None)
    ground_hyp = next((h for h in mht.hypotheses if h.parent_segment_id == "ground"), None)
    
    assert flyover_hyp is not None
    assert ground_hyp is not None
    assert flyover_hyp.score > ground_hyp.score * 5.0 # Flyover favored heavily

def test_horizontal_behavior_intact():
    # Just to confirm horizontal 2D logic still works completely ignoring Z
    seg_left = RoadSegment("left", (0, -10, 0), (100, -10, 0), has_elevation=False)
    seg_right = RoadSegment("right", (0, 10, 0), (100, 10, 0), has_elevation=False)
    
    matcher = MapMatcher([seg_left, seg_right], cross_track_variance=1.0)
    
    # Vehicle is on the right road
    ins = create_base_ins()
    ins.state.position_m = (0.0, 9.0, 0.0) # Close to right road
    ins.covariance[0:3, 0:3] = np.eye(3)
    
    mht = MultiHypothesisTracker(matcher)
    mht.initialize(ins)
    mht.update_map()
    
    dom = mht.get_dominant_hypothesis()
    assert dom.parent_segment_id == "right"
    # left should be rejected due to 19m horizontal error
    left_hyp = next((h for h in mht.hypotheses if h.parent_segment_id == "left"), None)
    assert left_hyp is None

def test_elevation_rejection():
    # If the elevation matches poorly and goes beyond the gate, it should be gated out entirely
    seg_flyover = RoadSegment("flyover", (0, 0, -50.0), (100, 0, -50.0), has_elevation=True)
    
    matcher = MapMatcher([seg_flyover], cross_track_variance=1.0, vertical_variance=1.0)
    
    # Vehicle is at ground (z=0)
    ins = create_base_ins(z_pos=0.0)
    ins.covariance[2, 2] = 1.0 # Tiny uncertainty
    
    mht = MultiHypothesisTracker(matcher)
    mht.initialize(ins)
    mht.update_map()
    
    # 50m error with 2.0 total variance -> M = 50 / 1.41 = 35. Rejected!
    flyover_hyp = next((h for h in mht.hypotheses if h.parent_segment_id == "flyover"), None)
    assert flyover_hyp is None
    # Only the unconstrained baseline should survive
    assert len(mht.hypotheses) == 1
    assert mht.hypotheses[0].parent_segment_id is None

def test_flyover_ambiguity_coexistence_and_resolution():
    # Tests that ambiguity allows coexistence, and evidence resolves it natively via Mahalanobis pruning
    seg_ground = RoadSegment("ground", (0, 0, 0.0), (100, 0, 0.0), has_elevation=True)
    seg_flyover = RoadSegment("flyover", (0, 0, -15.0), (100, 0, -15.0), has_elevation=True)
    
    matcher = MapMatcher([seg_ground, seg_flyover], cross_track_variance=1.0, vertical_variance=4.0)
    
    ins = create_base_ins(z_pos=-7.5) # Exactly in the middle between 0 and -15
    ins.covariance[2, 2] = 20.0 # High uncertainty
    
    mht = MultiHypothesisTracker(matcher, max_hypotheses=3)
    mht.initialize(ins)
    mht.update_map()
    
    # 1. Coexistence check
    # Check that both are within hypotheses since uncertainty is high and mahalanobis dist is low
    hyp_ids = [h.parent_segment_id for h in mht.hypotheses]
    assert "ground" in hyp_ids
    assert "flyover" in hyp_ids
    
    # 2. Resolution check
    # Now simulate finding evidence that we are definitively at flyover altitude (-15.0)
    # We'll just directly run another update but inject a tiny state variance and new z pos for testing
    flyover_hyp = next(h for h in mht.hypotheses if h.parent_segment_id == "flyover")
    flyover_hyp.ins.state.position_m = (10.0, 0.0, -14.5)
    flyover_hyp.ins.covariance[2, 2] = 1.0
    
    ground_hyp = next(h for h in mht.hypotheses if h.parent_segment_id == "ground")
    ground_hyp.ins.state.position_m = (10.0, 0.0, -14.5) 
    ground_hyp.ins.covariance[2, 2] = 1.0
    
    mht.update_map()
    
    # Ground should be completely gated out/rejected for new matches, remaining only as decayed baseline (or pruned)
    new_hyp_ids = [h.parent_segment_id for h in mht.hypotheses]
    # 'flyover' hypothesis perfectly matched and should be dominant
    dom = mht.get_dominant_hypothesis()
    assert dom.parent_segment_id == "flyover"
    assert dom.score > 0.3 
