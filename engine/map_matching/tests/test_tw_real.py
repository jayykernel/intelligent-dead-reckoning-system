"""
Requirement 3: Test Two-Wheeler Profile on Real Data.
Check how well it snaps and estimate its real accuracy.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.map_matching.road_network import RoadNetwork
from engine.map_matching.hmm_matcher import HMMMapMatcher

def validate_tw_real_data(session):
    print(f"\n" + "="*70)
    print(f"REQUIREMENT 3: Real Two-Wheeler Accuracy Validation ({session})")
    print("="*70)

    raw_root = "data/raw"
    synced = load_two_wheeler_session(os.path.join(raw_root, "two_wheeler"), session)

    lat = synced["gt_lat"].dropna().values
    lon = synced["gt_lon"].dropna().values
    alt = synced["gt_alt"].dropna().values # Assume if exists, else 0
    if len(alt) == 0: alt = np.zeros_like(lat)

    lat0, lon0, alt0 = lat[0], lon[0], alt[0]
    e_gt, n_gt, u_gt = latlon_to_enu(lat, lon, alt, lat0, lon0, alt0)
    gt_pos = np.column_stack([e_gt, n_gt, u_gt])

    # Load OSM road network for TW
    osm_path = os.path.join(raw_root, "tw_osm_extract.json")
    if not os.path.exists(osm_path):
        print(f"  OSM extract not found at {osm_path}, skipping.")
        return

    road_network = RoadNetwork(lat0=lat0, lon0=lon0, alt0=alt0)
    road_network.load_from_osm_json(osm_path)

    # Add simulated noise (5m) to TW trajectory
    np.random.seed(42)
    noise = np.random.normal(0, 5.0, size=(len(gt_pos), 2))
    noisy_pos = np.copy(gt_pos)
    noisy_pos[:, :2] += noise

    # Match
    matcher = HMMMapMatcher(road_network=road_network, vehicle_type="two_wheeler")
    # No heading available for TW - matcher handles this internally
    results = matcher.match_trajectory(noisy_pos)

    # Real accuracy: snapped point to GT
    errors_snapped_to_gt = []
    errors_raw_to_gt = []
    snapped_count = 0
    for i in range(len(results)):
        if results[i].snapped:
            snapped_count += 1
            err_snapped = np.linalg.norm(results[i].snapped_pos[:2] - gt_pos[i, :2])
            err_raw = np.linalg.norm(noisy_pos[i, :2] - gt_pos[i, :2])
            errors_snapped_to_gt.append(err_snapped)
            errors_raw_to_gt.append(err_raw)

    print(f"  Results ({snapped_count}/{len(results)} points snapped):")

    if errors_snapped_to_gt:
        print(f"\n  Snapped Points to GT Position (Real Accuracy):")
        print(f"    Mean:   {np.mean(errors_snapped_to_gt):.2f} m")
        print(f"    Median: {np.median(errors_snapped_to_gt):.2f} m")
        print(f"    Std:    {np.std(errors_snapped_to_gt):.2f} m")

        print(f"\n  Raw Points to GT Position (Before Snapping):")
        print(f"    Mean:   {np.mean(errors_raw_to_gt):.2f} m")
        print(f"    Median: {np.median(errors_raw_to_gt):.2f} m")
        print(f"    Std:    {np.std(errors_raw_to_gt):.2f} m")
    else:
        print("\n  No points snapped!")

if __name__ == "__main__":
    validate_tw_real_data("session1")
    validate_tw_real_data("session2")
