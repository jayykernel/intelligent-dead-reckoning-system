"""
Phase 16: Multi-Hypothesis Navigation.
Maintains branched parallel states for ambiguous map matching.
"""
from dataclasses import dataclass
from typing import List, Optional
import copy
import numpy as np

from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.map.matcher import MapMatcher, MapMatchResult
from core.sensors.data_types import ImuSample

@dataclass
class Hypothesis:
    id: str
    ins: StrapdownINS
    eskf: ErrorStateKalmanFilter
    score: float
    parent_segment_id: Optional[str] = None

class MultiHypothesisTracker:
    """
    Maintains multiple navigation hypotheses handling road structure ambiguities.
    Uses continuous map-match scoring mathematically coupled with Mahalanobis gates.
    """
    def __init__(
        self, 
        map_matcher: MapMatcher, 
        max_hypotheses: int = 3,
        unconstrained_baseline_prob: float = 0.5
    ):
        self.matcher = map_matcher
        self.max_hypotheses = max_hypotheses
        self.unconstrained_baseline_prob = unconstrained_baseline_prob
        self.hypotheses: List[Hypothesis] = []
        self.next_id = 1
        
    def initialize(self, base_ins: StrapdownINS):
        ins_clone = base_ins.clone()
        eskf_clone = ErrorStateKalmanFilter(ins_clone)
        self.hypotheses = [Hypothesis(f"H{self.next_id}", ins_clone, eskf_clone, 1.0)]
        self.next_id += 1
        
    def propagate(self, imu: ImuSample):
        for hyp in self.hypotheses:
            hyp.ins.propagate(imu, propagate_covariance=True)
            
    def update_map(self):
        """
        Evaluate map geometry against all hypotheses and branch.
        Score branching probabilities by Mahalanobis distance & map confidence.
        """
        new_hypotheses = []
        
        for hyp in self.hypotheses:
            matches = self.matcher.match_all(hyp.ins.state)
            
            # Baseline (unconstrained) hypothesis branch
            # Prevents a catastrophically wrong map match from seizing the state
            b_ins = hyp.ins.clone()
            baseline_hyp = Hypothesis(
                id=f"H{self.next_id}",
                ins=b_ins,
                eskf=ErrorStateKalmanFilter(b_ins),
                score=hyp.score * self.unconstrained_baseline_prob,
                parent_segment_id=hyp.parent_segment_id
            )
            self.next_id += 1
            new_hypotheses.append(baseline_hyp)
            
            for match in matches:
                child_ins = hyp.ins.clone()
                child_eskf = ErrorStateKalmanFilter(child_ins)
                
                # Apply map constraint
                res = child_eskf.update_position(match.projected_pos_ned, match.observation_cov, gate=4.0)
                
                if res.accepted:
                    # Score drops exponentially with Mahalanobis distance
                    # match.confidence penalizes distance constraints radially
                    obs_score = match.confidence * np.exp(-0.5 * res.mahalanobis_dist**2)
                    new_score = hyp.score * obs_score
                    
                    new_hyp = Hypothesis(
                        id=f"H{self.next_id}", 
                        ins=child_ins, 
                        eskf=child_eskf, 
                        score=new_score,
                        parent_segment_id=match.matched_segment.id
                    )
                    self.next_id += 1
                    new_hypotheses.append(new_hyp)
                    
        if not new_hypotheses:
            return
            
        # Pruning: Normalize scores
        total_score = sum(h.score for h in new_hypotheses)
        if total_score > 0:
            for h in new_hypotheses:
                h.score /= total_score
                
        # Sort by score descending and prune to top-K limits
        new_hypotheses.sort(key=lambda h: h.score, reverse=True)
        self.hypotheses = new_hypotheses[:self.max_hypotheses]
        
        # Prevent degenerate low-scores by renormalizing post-prune
        total_k_score = sum(h.score for h in self.hypotheses)
        if total_k_score > 0:
            for h in self.hypotheses:
                h.score /= total_k_score
                
    def get_dominant_hypothesis(self) -> Hypothesis:
        """Returns the highest scoring currently tracked navigation state."""
        return max(self.hypotheses, key=lambda h: h.score)
