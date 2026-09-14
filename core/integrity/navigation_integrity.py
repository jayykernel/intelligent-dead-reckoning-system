from enum import Enum
from dataclasses import dataclass
import numpy as np
from collections import deque
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import UpdateResult
from typing import Tuple

class IntegrityStatus(Enum):
    HEALTHY = 0    # Full confidence, tight bounds, reliable updates
    DEGRADED = 1   # High uncertainty (e.g. drifting during GNSS outage) but no explicit failures
    UNSAFE = 2     # Systematically lost: extreme covariance bounds or continuous measurement rejection

@dataclass
class SystemUncertainty:
    pos_std_m: float
    vel_std_mps: float
    att_std_deg: float

class NavigationIntegrityMonitor:
    """
    Evaluates absolute navigation confidence isolating 'highly uncertain' states 
    from 'highly confident but wrong' (overconfident) states.
    """
    def __init__(
        self, 
        pos_degraded_m: float = 10.0,
        pos_unsafe_m: float = 50.0,
        vel_degraded_mps: float = 2.0,
        vel_unsafe_mps: float = 10.0,
        att_degraded_deg: float = 5.0,
        att_unsafe_deg: float = 15.0,
        reject_window_size: int = 10,
        unsafe_reject_threshold: float = 0.5
    ):
        self.pos_degraded_m = pos_degraded_m
        self.pos_unsafe_m = pos_unsafe_m
        self.vel_degraded_mps = vel_degraded_mps
        self.vel_unsafe_mps = vel_unsafe_mps
        self.att_degraded_deg = att_degraded_deg
        self.att_unsafe_deg = att_unsafe_deg
        
        self.unsafe_reject_threshold = unsafe_reject_threshold
        self.rejection_history = deque(maxlen=reject_window_size)

    def report_update(self, result: UpdateResult) -> None:
        """
        Record the statistical acceptance/rejection of a measurement update.
        Continuous rejections indicate an overconfident, diverging navigation state.
        """
        self.rejection_history.append(not result.accepted)

    def assess(self, ins: StrapdownINS) -> Tuple[IntegrityStatus, SystemUncertainty]:
        """
        Evaluate structural filter bounds against measurement history tracking real-world limits.
        """
        cov = ins.covariance
        
        # Calculate 3D RMSE scalar bounds from block diagonals
        pos_var = float(np.trace(cov[0:3, 0:3]))
        vel_var = float(np.trace(cov[3:6, 3:6]))
        att_var = float(np.trace(cov[6:9, 6:9]))
        
        pos_std = float(np.sqrt(max(0.0, pos_var)))
        vel_std = float(np.sqrt(max(0.0, vel_var)))
        att_std = float(np.degrees(np.sqrt(max(0.0, att_var))))
        
        unc = SystemUncertainty(pos_std, vel_std, att_std)
        
        # 1. Catch "high confidence but wrong" (filter rejecting valid physical bounds continuously)
        rejection_rate = sum(self.rejection_history) / len(self.rejection_history) if self.rejection_history else 0.0
        if rejection_rate >= self.unsafe_reject_threshold and len(self.rejection_history) >= 3:
            return IntegrityStatus.UNSAFE, unc
            
        # 2. Hard limits (statistically lost regardless of update conditions)
        if (pos_std >= self.pos_unsafe_m or 
            vel_std >= self.vel_unsafe_mps or 
            att_std >= self.att_unsafe_deg):
            return IntegrityStatus.UNSAFE, unc
            
        # 3. Soft warnings (drifting heavily inside extended outages, e.g. tunnels)
        if (pos_std >= self.pos_degraded_m or 
            vel_std >= self.vel_degraded_mps or 
            att_std >= self.att_degraded_deg):
            return IntegrityStatus.DEGRADED, unc
            
        # 4. Safe
        return IntegrityStatus.HEALTHY, unc

    def reset(self) -> None:
        """Clear historical evaluation state."""
        self.rejection_history.clear()
