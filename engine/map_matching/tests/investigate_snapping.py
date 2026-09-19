"""
Investigate why snapping worsened accuracy.
Check if GT position is on the road or off-road.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu
from engine.map_matching.road_network import RoadNetwork


def check_gt_road_proximity():
    """Check how far GT positions are from nearest road."""
    print("\n" + "="*70)
    print("INVESTIGATION: GT Position Distance from OSM Roads")
    print("="*70)

    raw_root = "data/raw"
    s_df, v_df = load_iovnbd_session(raw_root, "S (Driver A)", "S4")
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)
    gt_pos = np.column_stack([e_gt, n_gt, u_gt])

    # Load road network
    osm_path = os.path.join("data/raw", "S4_osm_extract.json")
    road_network = RoadNetwork(lat0=lat0, lon0=lon0, alt0=alt0)
    road_network.load_from_osm_json(osm_path)

    # Check GT distance to nearest road (first 1000 points)
    distances = []
    for i in range(min(1000, len(gt_pos))):
        candidates = road_network.find_candidate_segments(gt_pos[i], radius_m=100.0, max_candidates=5)
        if candidates:
            distances.append(candidates[0][2])  # Closest distance
        else:
            distances.append(100.0)  # No road within 100m

    distances = np.array(distances)

    print(f"\n  GT Position Distance to Nearest OSM Road (first 1000 points):")
    print(f"    Mean:   {np.mean(distances):.2f} m")
    print(f"    Median: {np.median(distances):.2f} m")
    print(f"    Std:    {np.std(distances):.2f} m")
    print(f"    Max:    {np.max(distances):.2f} m")
    print(f"    Points > 10m from road: {np.sum(distances > 10)} ({np.sum(distances > 10)/len(distances)*100:.1f}%)")
    print(f"    Points > 20m from road: {np.sum(distances > 20)} ({np.sum(distances > 20)/len(distances)*100:.1f}%)")

    if np.mean(distances) > 5.0:
        print(f"\n  DIAGNOSIS: GT positions average {np.mean(distances):.1f}m from nearest OSM road.")
        print(f"  This suggests either:")
        print(f"    1. OSM road data is incomplete for this area")
        print(f"    2. GT positions are off-road (parking lots, driveways)")
        print(f"    3. Road network topology doesn't match actual driven route")


if __name__ == "__main__":
    check_gt_road_proximity()
