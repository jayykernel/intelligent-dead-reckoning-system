# Phase 7 Results

## 1. Car (Session S4) Accuracy Re-Test
We evaluated the distance from the **snapped point to the ground truth** (rather than just the distance to the map centerline). 
- **Mean Error (Raw vs GT)**: 4.32 m
- **Max Error (Raw vs GT)**: 7.71 m
- **Mean Error (Snapped vs GT)**: 7.05 m
- **Max Error (Snapped vs GT)**: 9.66 m

*Note: The snapped accuracy is bounded by a ~4.6m inherent offset between the IO-VNBD recorded driven lane position and the OSM topological centerline.*

## 2. Two-Wheeler Profile Validation
- **Data Source**: The two-wheeler validation was run on **real two-wheeler session data**, specifically `session1` from `data/raw/two_wheeler/`.
- **Map Extraction**: This was matched against a real OSM coverage extract (`tw_osm_extract.json`) downloaded for the actual corresponding coordinates of that route in India.
- **Result**: Successfully loaded the real TW OSM extract and matched the `session1` trajectory.
  - **Mean Error (Raw vs GT)**: 5.40 m
  - **Max Error (Raw vs GT)**: 9.64 m
  - **Mean Error (Snapped vs GT)**: 6.05 m
  - **Max Error (Snapped vs GT)**: 10.54 m
