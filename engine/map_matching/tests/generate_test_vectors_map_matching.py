import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.map_matching.road_network import RoadNetwork, RoadSegment
from engine.map_matching.hmm_matcher import HMMMapMatcher

def generate():
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
    # Street 2: South-North along East = 50 from North = 0 to 100m (Connected at (50, 0))
    seg2 = RoadSegment(
        segment_id=1,
        osm_way_id=1002,
        p_start_enu=np.array([50.0, 0.0]),
        p_end_enu=np.array([50.0, 100.0]),
        highway_type="secondary",
        oneway=False
    )
    # Street 3: Parallel disconnected road at North = 60 from East = 0 to 100m
    seg3 = RoadSegment(
        segment_id=2,
        osm_way_id=1003,
        p_start_enu=np.array([0.0, 60.0]),
        p_end_enu=np.array([100.0, 60.0]),
        highway_type="residential",
        oneway=False
    )
    rn.segments = [seg1, seg2, seg3]
    rn._build_spatial_index()

    # Save road network segments definition
    segments_json = []
    for s in rn.segments:
        segments_json.append({
            "segment_id": s.segment_id,
            "osm_way_id": s.osm_way_id,
            "p_start": s.p_start.tolist(),
            "p_end": s.p_end.tolist(),
            "highway_type": s.highway_type,
            "oneway": s.oneway,
            "length": float(s.length),
            "bearing_deg": float(s.bearing_deg)
        })

    test_inputs = [
        # Car trajectory driving along Street 1 (E=20 to E=50, N=2.0)
        {"vehicle_type": "car", "pos": [20.0, 2.0, 0.0], "heading": 90.0, "desc": "car_drive_1"},
        {"vehicle_type": "car", "pos": [40.0, 1.5, 0.0], "heading": 90.0, "desc": "car_drive_2"},
        {"vehicle_type": "car", "pos": [50.0, 10.0, 0.0], "heading": 0.0, "desc": "car_turn_street2"},
        {"vehicle_type": "car", "pos": [50.0, 30.0, 0.0], "heading": 0.0, "desc": "car_drive_street2"},
        # Off-road point (> search_radius) -> NO_CANDIDATE_ROAD_IN_RADIUS
        {"vehicle_type": "car", "pos": [500.0, 500.0, 0.0], "heading": 0.0, "desc": "car_offroad"},
        # Exceeds car max_deviation (25m) -> NO_CANDIDATE_ROAD_IN_RADIUS
        {"vehicle_type": "car", "pos": [30.0, 28.0, 0.0], "heading": 90.0, "desc": "car_exceed_deviation"},
        # Two-wheeler relaxed tolerance (35m deviation from Street 1, accepted by TW because max_deviation=50m)
        {"vehicle_type": "two_wheeler", "pos": [30.0, 35.0, 0.0], "heading": 90.0, "desc": "tw_relaxed_1"},
        {"vehicle_type": "two_wheeler", "pos": [60.0, 32.0, 0.0], "heading": 90.0, "desc": "tw_relaxed_2"},
        {"vehicle_type": "two_wheeler", "pos": [500.0, 500.0, 0.0], "heading": 90.0, "desc": "tw_offroad"}
    ]

    out_results = []

    # Run car sequence
    matcher_car = HMMMapMatcher(rn, vehicle_type="car")
    for inp in test_inputs[:6]:
        pos = np.array(inp["pos"])
        heading = inp["heading"]
        res = matcher_car.match_point(pos, heading_deg=heading)
        out_results.append({
            "desc": inp["desc"],
            "snapped": res.snapped,
            "snapped_pos": res.snapped_pos.tolist(),
            "confidence": float(res.confidence),
            "matched_segment_id": res.matched_segment_id,
            "osm_way_id": res.osm_way_id,
            "cross_track_error_m": float(res.cross_track_error_m),
            "fallback_reason": res.fallback_reason
        })

    # Run TW sequence
    matcher_tw = HMMMapMatcher(rn, vehicle_type="two_wheeler")
    for inp in test_inputs[6:]:
        pos = np.array(inp["pos"])
        heading = inp["heading"]
        res = matcher_tw.match_point(pos, heading_deg=heading)
        out_results.append({
            "desc": inp["desc"],
            "snapped": res.snapped,
            "snapped_pos": res.snapped_pos.tolist(),
            "confidence": float(res.confidence),
            "matched_segment_id": res.matched_segment_id,
            "osm_way_id": res.osm_way_id,
            "cross_track_error_m": float(res.cross_track_error_m),
            "fallback_reason": res.fallback_reason
        })

    out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "vectors"))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "map_matcher_network.json"), "w") as f:
        json.dump(segments_json, f, indent=2)
    with open(os.path.join(out_dir, "map_matcher_in.json"), "w") as f:
        json.dump(test_inputs, f, indent=2)
    with open(os.path.join(out_dir, "map_matcher_out.json"), "w") as f:
        json.dump(out_results, f, indent=2)
    print("Map matcher test vectors generated successfully in", out_dir)

if __name__ == "__main__":
    generate()
