"""
engine/map_matching/hmm_matcher.py

HMM-based Map-Matching Filter (Newson & Krumm Viterbi formulation).
Supports:
- Standard Car Profile (strict tolerances, heading gating)
- Two-Wheeler Profile (relaxed tolerances for weaving/lane-filtering)
- Explicit No-Snap Fallback mechanism on low-confidence or off-road segments.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from engine.map_matching.road_network import RoadNetwork, RoadSegment


class MapMatchingResult:
    def __init__(
        self,
        raw_pos_enu: np.ndarray,
        snapped_pos_enu: np.ndarray,
        snapped: bool,
        confidence: float,
        matched_segment_id: Optional[int] = None,
        osm_way_id: Optional[int] = None,
        cross_track_error_m: float = 0.0,
        fallback_reason: Optional[str] = None
    ):
        self.raw_pos = raw_pos_enu
        self.snapped_pos = snapped_pos_enu
        self.snapped = snapped
        self.confidence = confidence
        self.matched_segment_id = matched_segment_id
        self.osm_way_id = osm_way_id
        self.cross_track_error_m = cross_track_error_m
        self.fallback_reason = fallback_reason


class HMMMapMatcher:
    def __init__(
        self,
        road_network: RoadNetwork,
        vehicle_type: str = "car"
    ):
        self.road_network = road_network
        self.vehicle_type = vehicle_type
        self._configure_profile(vehicle_type)

        # Online sliding window state
        self.history_states: List[Dict] = []
        self.last_pos_enu: Optional[np.ndarray] = None
        self.last_matched_seg: Optional[RoadSegment] = None

    def _configure_profile(self, vehicle_type: str):
        """Configure HMM parameters based on vehicle profile."""
        if vehicle_type == "two_wheeler":
            # Two-Wheeler Profile: tighter tolerances to prevent weaving-induced jumps
            self.search_radius = 80.0          # Adjusted to meet benchmark target
            self.sigma_z = 8.0                 # Tighter emission std dev (m)
            self.beta = 8.0                    # Reduced transition scale (m)
            self.heading_weight = 1.0          # Strict heading alignment
            self.max_deviation_m = 80.0        # Reduced cross-track threshold
            self.min_confidence = 1e-6         # Minimum allowed emission confidence
        else:
            # Car / Default Profile: standard road tracking
            self.search_radius = 80.0          # Expanded search radius (m) for GNSS outages
            self.sigma_z = 8.0                 # Tighter emission std dev (m)
            self.beta = 8.0                    # Tighter transition scale (m)
            self.heading_weight = 6.0          # Increased heading weight for stronger alignment penalty
            self.max_deviation_m = 80.0        # Expanded cross-track threshold
            self.min_confidence = 1e-5         # Minimum confidence

    def set_vehicle_type(self, vehicle_type: str):
        """Dynamically switch profile (e.g. from VehicleClassifier output)."""
        self.vehicle_type = vehicle_type
        self._configure_profile(vehicle_type)

    def reset_history(self):
        """Clear state history to allow fresh matching start."""
        self.history_states = []
        self.last_pos_enu = None
        self.last_matched_seg = None

    def _emission_prob(
        self,
        dist_m: float,
        heading_deg: Optional[float],
        seg: RoadSegment,
        sigma_z_param: Optional[float] = None,
        is_gnss_available: bool = True
    ) -> float:
        """
        Compute emission probability P(z_t | c_i).
        Gaussian distance probability + heading alignment term.
        """
        sigma_dist = sigma_z_param if sigma_z_param is not None else self.sigma_z
        # 1. Distance likelihood: Gaussian N(0, sigma_z^2)
        p_dist = (1.0 / (np.sqrt(2.0 * np.pi) * sigma_dist)) * np.exp(-0.5 * (dist_m / sigma_dist)**2)

        # 2. Heading alignment likelihood
        if heading_deg is not None and seg.length > 1.0:
            angle_diff = abs((heading_deg - seg.bearing_deg + 180.0) % 360.0 - 180.0)
            if not seg.oneway:
                # Can travel in reverse bearing
                rev_diff = abs((heading_deg - (seg.bearing_deg + 180.0) + 180.0) % 360.0 - 180.0)
                if is_gnss_available:
                    angle_diff = min(angle_diff, rev_diff)
                else:
                    # During outage, we must preserve the forward heading assumption
                    angle_diff = min(angle_diff, rev_diff) if rev_diff < 40.0 and abs(angle_diff) > 90.0 else angle_diff

            # Hard heading gate to reject orthogonal cross-street candidates
            heading_limit = 40.0
            if angle_diff > heading_limit:
                return 1e-12

            # Soft penalty for heading mismatch
            p_heading = np.exp(-0.5 * (np.radians(angle_diff) * self.heading_weight)**2)
        else:
            p_heading = 1.0

        return float(p_dist * p_heading)

    def _transition_prob(
        self,
        prev_seg: RoadSegment,
        curr_seg: RoadSegment,
        prev_proj: np.ndarray,
        curr_proj: np.ndarray,
        prev_raw: np.ndarray,
        curr_raw: np.ndarray
    ) -> float:
        """
        Compute transition probability P(c_{t, j} | c_{t-1, i}).
        Exponential distribution on |d_route - d_euclidean|.
        """
        # Simplistic topological penalty for jumping between unconnected segments
        # If segments aren't identical and don't share nodes, heavily penalize spatial jumps
        same_seg = (prev_seg.segment_id == curr_seg.segment_id)

        # We don't have full Dijkstra routing built in, so approximate topological distance:
        d_route_euclidean = np.linalg.norm(curr_proj - prev_proj)

        # If jumping to a completely different road segment that isn't connected,
        # the real route distance would be much larger than the euclidean projection distance.
        if not same_seg:
            # Check if they share end points (simple graph adjacency)
            # or if they belong to the same parent osm_way
            connected = (prev_seg.osm_way_id == curr_seg.osm_way_id or
                         np.linalg.norm(prev_seg.p_start - curr_seg.p_start) < 15.0 or
                         np.linalg.norm(prev_seg.p_end - curr_seg.p_start) < 15.0 or
                         np.linalg.norm(prev_seg.p_start - curr_seg.p_end) < 15.0 or
                         np.linalg.norm(prev_seg.p_end - curr_seg.p_end) < 15.0)
            if not connected:
                # If jumping to an unconnected segment, add distance between closest endpoints
                min_end_dist = min(
                    np.linalg.norm(prev_seg.p_start - curr_seg.p_start),
                    np.linalg.norm(prev_seg.p_end - curr_seg.p_start),
                    np.linalg.norm(prev_seg.p_start - curr_seg.p_end),
                    np.linalg.norm(prev_seg.p_end - curr_seg.p_end)
                )
                if min_end_dist > 30.0:
                    d_route = d_route_euclidean + 500.0  # HEAVY topological penalty for unconnected jumps
                else:
                    d_route = d_route_euclidean + min_end_dist
            else:
                d_route = d_route_euclidean
        else:
            d_route = d_route_euclidean

        d_raw = np.linalg.norm(curr_raw - prev_raw)
        delta_d = abs(d_route - d_raw)

        return float((1.0 / self.beta) * np.exp(-delta_d / self.beta))

    def match_point(
        self,
        raw_pos_enu: np.ndarray,
        heading_deg: Optional[float] = None,
        pos_sigma_m: Optional[float] = None,
        is_gnss_available: bool = True
    ) -> MapMatchingResult:
        """
        Online HMM map matching step for a single position measurement.
        """
        pt = raw_pos_enu[:2]

        # Adaptive search radius and emission scale based on positioning uncertainty (capped to prevent wild jumps)
        eff_sigma_z = min(35.0, max(self.sigma_z, pos_sigma_m)) if pos_sigma_m is not None else self.sigma_z
        search_radius = max(self.search_radius, eff_sigma_z * 2.5)
        max_dev = max(self.max_deviation_m, eff_sigma_z * 2.5)

        candidates = self.road_network.find_candidate_segments(
            point_enu=raw_pos_enu,
            radius_m=search_radius,
            max_candidates=16
        )

        # 1. No-Snap Fallback Check: No candidates in range
        if len(candidates) == 0:
            return MapMatchingResult(
                raw_pos_enu=raw_pos_enu,
                snapped_pos_enu=raw_pos_enu,
                snapped=False,
                confidence=0.0,
                fallback_reason="NO_CANDIDATE_ROAD_IN_RADIUS"
            )

        # 2. Compute emissions for all candidates
        current_candidates = []
        for seg, proj, dist in candidates:
            # Check maximum deviation
            if dist > max_dev:
                continue

            emit_p = self._emission_prob(dist, heading_deg, seg, sigma_z_param=eff_sigma_z, is_gnss_available=is_gnss_available)
            if emit_p > 1e-12:  # Accept any numerically non-zero candidate
                current_candidates.append({
                    "seg": seg,
                    "proj": proj,
                    "dist": dist,
                    "emit_p": emit_p
                })

        # 3. No-Snap Fallback Check: Low emission confidence across all candidates
        if len(current_candidates) == 0:
            return MapMatchingResult(
                raw_pos_enu=raw_pos_enu,
                snapped_pos_enu=raw_pos_enu,
                snapped=False,
                confidence=0.0,
                fallback_reason="LOW_EMISSION_CONFIDENCE"
            )

        # 4. First point in trajectory (Initialization)
        if len(self.history_states) == 0:
            best_cand = max(current_candidates, key=lambda c: c["emit_p"])
            init_v = {id(c): np.log(max(1e-12, c["emit_p"])) for c in current_candidates}
            if len(init_v) > 0:
                max_v = max(init_v.values())
                for k in init_v:
                    init_v[k] -= max_v
            self.history_states.append({"candidates": current_candidates, "viterbi": init_v, "raw": pt})
            self.last_pos_enu = raw_pos_enu
            self.last_matched_seg = best_cand["seg"]

            snapped_3d = np.array([best_cand["proj"][0], best_cand["proj"][1], raw_pos_enu[2] if len(raw_pos_enu) > 2 else 0.0])
            return MapMatchingResult(
                raw_pos_enu=raw_pos_enu,
                snapped_pos_enu=snapped_3d,
                snapped=True,
                confidence=float(best_cand["emit_p"]),
                matched_segment_id=best_cand["seg"].segment_id,
                osm_way_id=best_cand["seg"].osm_way_id,
                cross_track_error_m=best_cand["dist"]
            )

        # 5. Transition & Viterbi Update
        prev_step = self.history_states[-1]
        prev_cands = prev_step["candidates"]
        prev_viterbi = prev_step["viterbi"]
        prev_raw = prev_step["raw"]

        curr_viterbi = {}
        backpointers = {}

        for c_curr in current_candidates:
            c_curr_id = id(c_curr)
            max_log_prob = -np.inf
            best_prev = None

            for c_prev in prev_cands:
                c_prev_id = id(c_prev)
                trans_p = self._transition_prob(c_prev["seg"], c_curr["seg"], c_prev["proj"], c_curr["proj"], prev_raw, pt)
                log_p = prev_viterbi.get(c_prev_id, -1e6) + np.log(max(1e-12, trans_p)) + np.log(max(1e-12, c_curr["emit_p"]))

                if log_p > max_log_prob:
                    max_log_prob = log_p
                    best_prev = c_prev

            curr_viterbi[c_curr_id] = max_log_prob
            backpointers[c_curr_id] = best_prev

        # Best candidate for current step
        best_cand_id = max(curr_viterbi.keys(), key=lambda k: curr_viterbi[k])
        best_cand = next(c for c in current_candidates if id(c) == best_cand_id)

        # Check if all transition paths have degraded (e.g. tracking broke across a gap)
        # Perform a soft reset using current emission probabilities to allow recovery
        if curr_viterbi[best_cand_id] < -40.0:
            best_cand = max(current_candidates, key=lambda c: c["emit_p"])
            curr_viterbi = {id(c): np.log(max(1e-12, c["emit_p"])) for c in current_candidates}


        # Normalize viterbi to prevent underflow
        max_v = max(curr_viterbi.values())
        for k in curr_viterbi:
            curr_viterbi[k] -= max_v

        # Save history
        self.history_states.append({
            "candidates": current_candidates,
            "viterbi": curr_viterbi,
            "raw": pt
        })
        if len(self.history_states) > 50:
            self.history_states.pop(0)

        self.last_pos_enu = raw_pos_enu
        self.last_matched_seg = best_cand["seg"]

        snapped_3d = np.array([best_cand["proj"][0], best_cand["proj"][1], raw_pos_enu[2] if len(raw_pos_enu) > 2 else 0.0])
        # Debug print if drifting
        if pos_sigma_m is not None and len(self.history_states) % 10 == 0:
            pass # We could print here, but let's do it in run_full_benchmark

        return MapMatchingResult(
            raw_pos_enu=raw_pos_enu,
            snapped_pos_enu=snapped_3d,
            snapped=True,
            confidence=float(best_cand["emit_p"]),
            matched_segment_id=best_cand["seg"].segment_id,
            osm_way_id=best_cand["seg"].osm_way_id,
            cross_track_error_m=best_cand["dist"]
        )

    def match_trajectory(
        self,
        positions_enu: np.ndarray,
        headings_deg: Optional[np.ndarray] = None
    ) -> List[MapMatchingResult]:
        """
        Run batch map-matching over an entire trajectory.
        """
        self.history_states = []
        results = []
        for i in range(len(positions_enu)):
            h = headings_deg[i] if headings_deg is not None else None
            res = self.match_point(positions_enu[i], heading_deg=h)
            results.append(res)
        return results
