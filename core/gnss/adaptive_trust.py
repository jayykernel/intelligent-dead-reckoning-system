import numpy as np
from enum import Enum, auto

class UncertaintyCalibrator:
    """
    Recalibrates raw model log_var to empirical error bounds to prevent
    overconfident updates from disrupting the ESKF.
    """
    def __init__(self, base_variance_floor: float = 2.0, variance_scale: float = 2.068):
        self.base_variance_floor = base_variance_floor
        self.variance_scale = variance_scale
        
    def calibrate(self, raw_variance: float) -> float:
        """Apply empirical scaling to the model's self-reported variance."""
        return self.base_variance_floor + (raw_variance * self.variance_scale)

class TrustState(Enum):
    NORMAL = auto()
    CAUTION = auto()
    FALLBACK = auto()

class AdaptiveTrustMonitor:
    """
    Hysteretic state machine and confidence decay monitor.
    Uses temporal autocorrelation (sustained biased innovations) to detect
    distribution shifts and safely inflate variance / trigger fallback.
    """
    def __init__(self, 
                 bias_window_size: int = 10,
                 caution_bias_threshold: float = 2.5,
                 fallback_bias_threshold: float = 4.0):
                 
        self.state = TrustState.NORMAL
        self.trust_score = 1.0
        
        self.bias_window_size = bias_window_size
        self.caution_bias_threshold = caution_bias_threshold
        self.fallback_bias_threshold = fallback_bias_threshold
        
        self.recent_innovations = []
        
    def reset(self):
        """Reset when GNSS is available."""
        self.state = TrustState.NORMAL
        self.trust_score = 1.0
        self.recent_innovations.clear()

    def evaluate_update(self, innovation: float) -> float:
        """
        Evaluate innovation bias and return a trust-mapped variance multiplier.
        Multiplier = 1.0 means full trust. Higher means less trust.
        Multiplier = infinity means gate is closed (fallback).
        """
        self.recent_innovations.append(innovation)
        if len(self.recent_innovations) > self.bias_window_size:
            self.recent_innovations.pop(0)
            
        # 1. Persistent Bias Detection
        mean_abs_bias = abs(np.mean(self.recent_innovations)) if len(self.recent_innovations) > 0 else 0.0
        
        # 2. Hysteretic State Machine
        if self.state == TrustState.NORMAL:
            if mean_abs_bias >= self.fallback_bias_threshold:
                self.state = TrustState.FALLBACK
            elif mean_abs_bias >= self.caution_bias_threshold:
                self.state = TrustState.CAUTION
                
        elif self.state == TrustState.CAUTION:
            if mean_abs_bias >= self.fallback_bias_threshold:
                self.state = TrustState.FALLBACK
            elif mean_abs_bias < self.caution_bias_threshold - 0.5: # Hysteresis
                self.state = TrustState.NORMAL
                
        elif self.state == TrustState.FALLBACK:
            if mean_abs_bias < self.caution_bias_threshold - 1.0: # Deep hysteresis for recovery
                self.state = TrustState.CAUTION

        # 3. Trust Score Map
        if self.state == TrustState.NORMAL:
            self.trust_score = 1.0
            return 1.0
        elif self.state == TrustState.CAUTION:
            self.trust_score = 0.5
            return 10.0 # 10x variance penalty (soft fallback)
        else: # FALLBACK
            self.trust_score = 0.0
            return float('inf') # Complete rejection
