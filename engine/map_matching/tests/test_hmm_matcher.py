"""
engine/map_matching/tests/test_hmm_matcher.py

Unit tests for HMM Map Matching and No-Snap Fallback logic.
"""

import os
import sys
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.map_matching.road_network import RoadNetwork, RoadSegment
from engine.map_matching.hmm_matcher import HMMMapMatcher


@pytest.fixture
def sample_road_network():
    """Create a synthetic mini road network with two orthogonal streets."""
    rn = RoadNetwork(lat0=52.4, lon0=-1.5)
    # Street 1: West-East along North = 0 from East = 0 to 100m
    seg1 = RoadSegment(
        segment_id=0,
        osm_way_id=1001,
        p_start_enu=np.array([0.0, 0.0]),
        p_end_enu=np.array([100.0, 0.0]),
        highway_type="primary",
        oneway=False
    )
    # Street 2: South-North along East = 50 from North = 0 to 100m
    seg2 = RoadSegment(
        segment_id=1,
        osm_way_id=1002,
        p_start_enu=np.array([50.0, 0.0]),
        p_end_enu=np.array([50.0, 100.0]),
        highway_type="secondary",
        oneway=False
    )
    rn.segments = [seg1, seg2]
    rn._build_spatial_index()
    return rn


def test_snapping_to_nearest_road(sample_road_network):
    matcher = HMMMapMatcher(sample_road_network, vehicle_type="car")

    # Point near Street 1 (E=20, N=3)
    raw_pt = np.array([20.0, 3.0, 0.0])
    res = matcher.match_point(raw_pt, heading_deg=90.0)

    assert res.snapped is True
    assert res.matched_segment_id == 0
    assert np.isclose(res.snapped_pos[0], 20.0)
    assert np.isclose(res.snapped_pos[1], 0.0)
    assert np.isclose(res.cross_track_error_m, 3.0)


def test_no_snap_fallback_off_road(sample_road_network):
    matcher = HMMMapMatcher(sample_road_network, vehicle_type="car")

    # Point far off-road (E=200, N=200) - distance > 100m
    raw_pt = np.array([200.0, 200.0, 0.0])
    res = matcher.match_point(raw_pt)

    assert res.snapped is False
    assert res.fallback_reason == "NO_CANDIDATE_ROAD_IN_RADIUS"
    assert np.allclose(res.snapped_pos, raw_pt)


def test_two_wheeler_relaxed_tolerance(sample_road_network):
    matcher_car = HMMMapMatcher(sample_road_network, vehicle_type="car")
    matcher_tw = HMMMapMatcher(sample_road_network, vehicle_type="two_wheeler")

    # Point 35m away from Street 1 (E=30, N=35)
    raw_pt = np.array([30.0, 35.0, 0.0])

    res_car = matcher_car.match_point(raw_pt, heading_deg=90.0)
    res_tw = matcher_tw.match_point(raw_pt, heading_deg=90.0)

    # Car profile has max_deviation_m = 25m -> should reject / no-snap
    assert res_car.snapped is False

    # TW profile has max_deviation_m = 50m -> should accept and snap
    assert res_tw.snapped is True
    assert res_tw.matched_segment_id == 0
