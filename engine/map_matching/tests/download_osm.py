import sys
import os
import requests
import json

def download_osm_extract(rect):
    """
    Downloads OSM road network via Overpass API.
    rect = (south, west, north, east)
    """
    overpass_url = "http://overpass-api.de/api/interpreter"
    # Filter for drivable roads (highways)
    overpass_query = f"""
    [out:json];
    (
      way["highway"]["highway"!="pedestrian"]["highway"!="footway"]["highway"!="path"]["highway"!="cycleway"]["highway"!="steps"]({rect[0]},{rect[1]},{rect[2]},{rect[3]});
    );
    (._;>;);
    out body;
    """
    print("Downloading OSM data (this may take a minute)...")
    headers = {'User-Agent': 'OfflineMapMatcher/1.0'}
    response = requests.post(overpass_url, data={'data': overpass_query}, headers=headers)

    if response.status_code == 200:
        data = response.json()
        print(f"Downloaded {len(data['elements'])} elements.")

        out_dir = os.path.join(os.path.dirname(__file__), "../../../data/raw")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "S4_osm_extract.json")
        with open(out_path, 'w') as f:
            json.dump(data, f)
        print(f"Saved to {out_path}")
    else:
        print(f"Error {response.status_code}: {response.text}")

if __name__ == "__main__":
    download_osm_extract((52.360, -1.600, 52.445, -1.420))
