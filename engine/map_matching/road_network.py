"""
engine/map_matching/road_network.py

Lightweight, self-contained Offline OSM Road Network parser and spatial index.
Converts geographic road segments into a queryable graph in local ENU or lat/lon coordinates.
"""

import os
import json
import numpy as np
from typing import List, Dict, Tuple, Optional
from scipy.spatial import KDTree

from training.data_loader import latlon_to_enu


class RoadSegment:
    def __init__(
        self,
        segment_id: int,
        osm_way_id: int,
        p_start_enu: np.ndarray,
        p_end_enu: np.ndarray,
        highway_type: str = "primary",
        oneway: bool = False
    ):
        self.segment_id = segment_id
        self.osm_way_id = osm_way_id
        self.p_start = p_start_enu[:2]  # [East, North]
        self.p_end = p_end_enu[:2]      # [East, North]
        self.highway_type = highway_type
        self.oneway = oneway

        self.vec = self.p_end - self.p_start
        self.length = float(np.linalg.norm(self.vec))
        if self.length > 1e-6:
            self.dir = self.vec / self.length
            # Bearing in degrees (0 = North, 90 = East, in ENU: atan2(East, North))
            self.bearing_deg = float(np.degrees(np.arctan2(self.dir[0], self.dir[1])) % 360)
        else:
            self.dir = np.zeros(2)
            self.bearing_deg = 0.0

        self.midpoint = 0.5 * (self.p_start + self.p_end)

    def project_point(self, point_enu: np.ndarray) -> Tuple[np.ndarray, float, float]:
        """
        Project a 2D ENU point onto this line segment.

        Returns:
        (projected_point_enu, perpendicular_dist_meters, along_track_ratio_0_to_1)
        """
        pt = point_enu[:2]
        if self.length < 1e-6:
            return self.p_start, float(np.linalg.norm(pt - self.p_start)), 0.0

        # Vector from start to point
        v = pt - self.p_start
        # Projection along segment
        t = float(np.dot(v, self.dir)) / self.length
        t_clamped = max(0.0, min(1.0, t))

        proj = self.p_start + t_clamped * self.vec
        perp_dist = float(np.linalg.norm(pt - proj))

        return proj, perp_dist, t_clamped


class RoadNetwork:
    def __init__(self, lat0: float, lon0: float, alt0: float = 0.0):
        self.lat0 = lat0
        self.lon0 = lon0
        self.alt0 = alt0

        self.segments: List[RoadSegment] = []
        self.kdtree: Optional[KDTree] = None
        self.midpoints: Optional[np.ndarray] = None

    def load_from_osm_json(self, osm_json_path: str):
        """
        Parse raw OSM Overpass JSON extract and build local ENU road segments.
        """
        with open(osm_json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        nodes = {}
        for elem in data.get("elements", []):
            if elem.get("type") == "node":
                lat = elem["lat"]
                lon = elem["lon"]
                e, n, u = latlon_to_enu(np.array([lat]), np.array([lon]), np.array([0.0]), self.lat0, self.lon0, self.alt0)
                nodes[elem["id"]] = np.array([float(e[0]), float(n[0])])

        segment_id = 0
        for elem in data.get("elements", []):
            if elem.get("type") == "way" and "nodes" in elem:
                way_nodes = elem["nodes"]
                tags = elem.get("tags", {})
                highway = tags.get("highway", "residential")
                oneway = tags.get("oneway", "no") in ["yes", "1", "true"]

                for i in range(len(way_nodes) - 1):
                    n1_id = way_nodes[i]
                    n2_id = way_nodes[i + 1]
                    if n1_id in nodes and n2_id in nodes:
                        p1 = nodes[n1_id]
                        p2 = nodes[n2_id]

                        vec = p2 - p1
                        length = float(np.linalg.norm(vec))
                        if length < 0.5:
                            continue

                        # Subdivide long segments into max 10m chunks for better tangent adherence on curves
                        num_chunks = max(1, int(np.ceil(length / 10.0)))
                        for j in range(num_chunks):
                            chunk_p1 = p1 + (j / num_chunks) * vec
                            chunk_p2 = p1 + ((j + 1) / num_chunks) * vec

                            seg = RoadSegment(
                                segment_id=segment_id,
                                osm_way_id=elem["id"],
                                p_start_enu=chunk_p1,
                                p_end_enu=chunk_p2,
                                highway_type=highway,
                                oneway=oneway
                            )
                            self.segments.append(seg)
                            segment_id += 1

        print(f"[RoadNetwork] Loaded {len(self.segments)} road segments from OSM extract.")
        self._build_spatial_index()

    def _build_spatial_index(self):
        """Build KDTree on segment midpoints for fast range querying."""
        if not self.segments:
            return
        self.midpoints = np.array([seg.midpoint for seg in self.segments])
        self.kdtree = KDTree(self.midpoints)

    def find_candidate_segments(
        self,
        point_enu: np.ndarray,
        radius_m: float = 30.0,
        max_candidates: int = 10
    ) -> List[Tuple[RoadSegment, np.ndarray, float]]:
        """
        Find candidate road segments within radius of point_enu.

        Returns:
        List of (RoadSegment, projected_point_enu, perpendicular_distance) sorted by distance.
        """
        if self.kdtree is None or len(self.segments) == 0:
            return []

        # Query KDTree with slightly larger radius to account for segment lengths
        pt = point_enu[:2]
        query_radius = radius_m + 50.0  # Max expected half-segment length
        indices = self.kdtree.query_ball_point(pt, r=query_radius)

        candidates = []
        for idx in indices:
            seg = self.segments[idx]
            proj, dist, t = seg.project_point(pt)
            if dist <= radius_m:
                candidates.append((seg, proj, dist))

        # Sort by perpendicular distance
        candidates.sort(key=lambda x: x[2])
        return candidates[:max_candidates]
