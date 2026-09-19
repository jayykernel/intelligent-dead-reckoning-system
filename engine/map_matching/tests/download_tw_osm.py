"""
Download OSM extracts for two-wheeler sessions in India (Bangalore region).
"""

import sys
import os
import requests
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

def download_osm_extract(rect, output_name):
    """
    Downloads OSM road network via Overpass API.
    rect = (south, west, north, east)
    """
    overpass_url = "http://overpass-api.de/api/interpreter"
    overpass_query = f"""
    [out:json];
    (
      way["highway"]["highway"!="pedestrian"]["highway"!="footway"]["highway"!="path"]["highway"!="cycleway"]["highway"!="steps"]({rect[0]},{rect[1]},{rect[2]},{rect[3]});
    );
    (._;>;);
    out body;
    """
    print(f"Downloading OSM data for {output_name} (this may take a minute)...")
    headers = {'User-Agent': 'OfflineMapMatcher/1.0'}
    response = requests.post(overpass_url, data={'data': overpass_query}, headers=headers)

    if response.status_code == 200:
        data = response.json()
        print(f"Downloaded {len(data['elements'])} elements.")

        out_dir = os.path.join(os.path.dirname(__file__), "../../../data/raw")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, output_name)
        with open(out_path, 'w') as f:
            json.dump(data, f)
        print(f"Saved to {out_path}")
        return True
    else:
        print(f"Error {response.status_code}: {response.text}")
        return False

if __name__ == "__main__":
    # Two-wheeler session coordinates (Bangalore, India)
    # session1: Lat [11.101036, 11.107315], Lon [77.346590, 77.355169]
    # session2: Lat [11.098852, 11.100215], Lon [77.353825, 77.355317]

    # Combined bounding box for both sessions
    download_osm_extract((11.098, 77.346, 11.108, 77.356), "tw_osm_extract.json")
