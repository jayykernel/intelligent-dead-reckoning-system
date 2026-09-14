import pytest
import numpy as np
import math
from core.map.geometry import RoadSegment
from core.map.matcher import MapMatcher
from core.navigation.state import NavState

def test_road_segment_geometry():
    seg = RoadSegment("r1", (0.0, 0.0, 0.0), (100.0, 0.0, 0.0)) # Straight North
    np.testing.assert_array_equal(seg.vector_2d, np.array([100.0, 0.0]))
    assert seg.length_2d == 100.0
    np.testing.assert_array_equal(seg.unit_direction_2d, np.array([1.0, 0.0]))

def create_state(p: tuple, v: tuple):
    return NavState(
        timestamp_ns=0, position_m=p, velocity_mps=v,
        attitude_q_v2n=(1.0,0.0,0.0,0.0), accel_bias_mps2=(0.0,0.0,0.0), gyro_bias_radps=(0.0,0.0,0.0)
    )

def test_map_matcher_basic_snap():
    seg = RoadSegment("r1", (0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    matcher = MapMatcher([seg])
    
    # State is 5m East of the road, moving North at 10m/s
    state = create_state((50.0, 5.0, 0.0), (10.0, 0.0, 0.0))
    
    res = matcher.match(state)
    assert res is not None
    assert res.matched_segment.id == "r1"
    
    # Point should be projected back to East=0
    assert abs(res.projected_pos_ned[0] - 50.0) < 1e-4
    assert abs(res.projected_pos_ned[1] - 0.0) < 1e-4
    assert res.cross_track_error == 5.0
    
    # Covariance check: Along track (North, index 0) should be 1000, Cross track (East, index 1) should be 1.0
    cov = res.observation_cov
    assert cov[0,0] == 1e9
    assert cov[1,1] == 1.0
    
def test_map_matcher_heading_rejection():
    seg = RoadSegment("r1", (0.0, 0.0, 0.0), (100.0, 0.0, 0.0)) # Road goes North
    matcher = MapMatcher([seg])
    
    # State is perfectly on road, but moving East (perpendicular)
    state = create_state((50.0, 0.0, 0.0), (0.0, 10.0, 0.0))
    
    # Should reject due to heading exceeding max_heading_error_rad
    res = matcher.match(state)
    assert res is None

def test_map_matcher_ambiguous_dist_rejection():
    seg = RoadSegment("r1", (0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    matcher = MapMatcher([seg])
    
    # State is 20m off the road, moving North
    state = create_state((50.0, 20.0, 0.0), (10.0, 0.0, 0.0))
    
    # Should reject due to > 15m distance
    res = matcher.match(state)
    assert res is None
