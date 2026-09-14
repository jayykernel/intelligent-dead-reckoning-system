import numpy as np
import math
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass
from core.map.geometry import RoadSegment
from core.navigation.state import NavState

@dataclass
class MapMatchResult:
    """Result of a map matching attempt."""
    matched_segment: RoadSegment
    projected_pos_ned: Tuple[float, float, float]
    cross_track_error: float
    confidence: float
    observation_cov: np.ndarray

class MapMatcher:
    """
    Uncertainty-aware map matching engine.
    Finds the most likely road segment and generates an ESKF position constraint.
    """
    def __init__(
        self, 
        segments: List[RoadSegment], 
        max_distance_m: float = 15.0,
        max_heading_error_rad: float = math.radians(45.0),
        cross_track_variance: float = 1.0,
        along_track_variance: float = 1e9,  # High variance to prevent snapping along-track
        vertical_variance: float = 1000.0
    ):
        self.segments = segments
        self.max_distance_m = max_distance_m
        self.max_heading_error_rad = max_heading_error_rad
        self.cross_track_variance = cross_track_variance
        self.along_track_variance = along_track_variance
        self.vertical_variance = vertical_variance
        
    def _point_segment_distance(self, p: np.ndarray, seg: RoadSegment) -> Tuple[np.ndarray, float, float]:
        """
        Projects point p onto the segment.
        Returns: 
            projected_point (3D)
            distance (2D)
            t (parameter along segment, 0 to 1)
        """
        p2d = p[0:2]
        start2d = np.array(seg.start_ned[0:2])
        end2d = np.array(seg.end_ned[0:2])
        
        v = end2d - start2d
        w = p2d - start2d
        
        length_sq = np.dot(v, v)
        if length_sq < 1e-6:
            # Degenerate point segment
            proj2d = start2d
            t = 0.0
        else:
            t = max(0.0, min(1.0, np.dot(w, v) / length_sq))
            proj2d = start2d + t * v
            
        dist = np.linalg.norm(p2d - proj2d)
        
        # 3D projection (keeping altitude same as state to ignore vertical snapping for now)
        proj3d = np.array([proj2d[0], proj2d[1], float(p[2])])
        return proj3d, float(dist), float(t)
        
    def match(self, state: NavState) -> Optional[MapMatchResult]:
        """
        Evaluate candidate segments and produce an ESKF measurement if valid.
        
        Basic heuristic: Closest segment that matches heading criteria.
        Precursor to Phase 16 multi-hypothesis tracker.
        """
        p_ned = np.array(state.position_m)
        v_ned = np.array(state.velocity_mps)
        v_2d = v_ned[0:2]
        speed = np.linalg.norm(v_2d)
        
        best_match = None
        min_dist = float('inf')
        
        # Determine movement heading if speed is sufficient, else None
        heading_rad = math.atan2(v_2d[1], v_2d[0]) if speed > 1.0 else None

        candidate_results = []

        for seg in self.segments:
            proj3d, dist, t = self._point_segment_distance(p_ned, seg)
            
            if dist > self.max_distance_m:
                continue
                
            # If we are moving, enforce heading consistency
            heading_penalty = 1.0 # 1.0 = perfect
            if heading_rad is not None:
                seg_dir = seg.unit_direction_2d
                seg_heading = math.atan2(seg_dir[1], seg_dir[0])
                
                # Minimum angle difference
                angle_diff = abs(math.atan2(math.sin(heading_rad - seg_heading), math.cos(heading_rad - seg_heading)))
                
                # Check reverse direction as well (two-way roads)
                angle_diff_rev = abs(math.atan2(math.sin(heading_rad - (seg_heading + math.pi)), math.cos(heading_rad - (seg_heading + math.pi))))
                
                best_angle_diff = min(angle_diff, angle_diff_rev)
                
                if best_angle_diff > self.max_heading_error_rad:
                    continue
                    
                # Normalize penalty so larger angle = lower confidence
                heading_penalty = max(0.0, 1.0 - (best_angle_diff / self.max_heading_error_rad))
                
            # Prefer closer distances and better headings
            candidate_results.append({
                "segment": seg,
                "proj3d": proj3d,
                "dist": dist,
                "heading_penalty": heading_penalty
            })
            
        if not candidate_results:
            return None
            
        # Select best match by distance and heading penalty
        # Score = dist / heading_penalty. Lower is better.
        # Add epsilon to prevent div by zero
        best_candidate = min(candidate_results, key=lambda c: c["dist"] / (c["heading_penalty"] + 0.01))
            
        # Construct covariance matrix prioritizing cross-track constraint
        seg = best_candidate["segment"]
        u_along = seg.unit_direction_2d
        # Cross track vector is orthogonal to along-track
        u_cross = np.array([-u_along[1], u_along[0]])
        
        # 3x3 Eigenvector matrix
        V = np.eye(3)
        V[0:2, 0] = u_along
        V[0:2, 1] = u_cross
        
        # Eigenvalue matrix
        Lambda = np.diag([self.along_track_variance, self.cross_track_variance, self.vertical_variance])
        
        # S = V * Lambda * V^T
        pos_cov = V @ Lambda @ V.T
        
        # Calculate overall confidence
        dist = best_candidate["dist"]
        confidence = max(0.0, 1.0 - (dist / self.max_distance_m)) * best_candidate["heading_penalty"]
        
        return MapMatchResult(
            matched_segment=seg,
            projected_pos_ned=tuple(best_candidate["proj3d"]),
            cross_track_error=dist,
            confidence=confidence,
            observation_cov=pos_cov
        )

    def match_all(self, state: NavState) -> List[MapMatchResult]:
        p_ned = np.array(state.position_m)
        v_ned = np.array(state.velocity_mps)
        v_2d = v_ned[0:2]
        speed = np.linalg.norm(v_2d)
        
        heading_rad = math.atan2(v_2d[1], v_2d[0]) if speed > 1.0 else None
        
        candidate_results = []
        for seg in self.segments:
            proj3d, dist, t = self._point_segment_distance(p_ned, seg)
            
            if dist > self.max_distance_m:
                continue
                
            heading_penalty = 1.0
            if heading_rad is not None:
                seg_dir = seg.unit_direction_2d
                seg_heading = math.atan2(seg_dir[1], seg_dir[0])
                angle_diff = abs(math.atan2(math.sin(heading_rad - seg_heading), math.cos(heading_rad - seg_heading)))
                angle_diff_rev = abs(math.atan2(math.sin(heading_rad - (seg_heading + math.pi)), math.cos(heading_rad - (seg_heading + math.pi))))
                best_angle_diff = min(angle_diff, angle_diff_rev)
                if best_angle_diff > self.max_heading_error_rad:
                    continue
                heading_penalty = max(0.0, 1.0 - (best_angle_diff / self.max_heading_error_rad))
                
            u_along = seg.unit_direction_2d
            u_cross = np.array([-u_along[1], u_along[0]])
            V = np.eye(3)
            V[0:2, 0] = u_along
            V[0:2, 1] = u_cross
            Lambda = np.diag([self.along_track_variance, self.cross_track_variance, self.vertical_variance])
            pos_cov = V @ Lambda @ V.T
            confidence = max(0.0, 1.0 - (dist / self.max_distance_m)) * heading_penalty
            
            res = MapMatchResult(
                matched_segment=seg,
                projected_pos_ned=tuple(proj3d),
                cross_track_error=dist,
                confidence=confidence,
                observation_cov=pos_cov
            )
            candidate_results.append(res)
            
        # Sort by confidence descending
        candidate_results.sort(key=lambda r: r.confidence, reverse=True)
        return candidate_results

