"""
Phase 14: Dynamic Bias / Error Adaptation.

Provides environmental error adaptation mapping real-time vibration (roughness)
into dynamic INS process noise variance scaling, preventing filter overconfidence 
during perturbed (bumpy or vibrating) motion.

Also provides Magnetic Reliability Scoring to dynamically down-weight magnetometer 
heading assistance during environmental anomalies (hard/soft iron distortion).
"""
import numpy as np
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from core.navigation.mechanization import StrapdownINS

class DynamicErrorAdapter:
    """
    Adapts INS process noise continuously based on high-frequency environmental 
    vibrations (roughness) and spectral stochasticity.
    """
    def __init__(
        self, 
        base_accel_noise: float = 0.05, 
        base_gyro_noise: float = 0.005,
        roughness_scale_accel: float = 0.1,
        roughness_scale_gyro: float = 0.01
    ):
        self.base_accel_noise = base_accel_noise
        self.base_gyro_noise = base_gyro_noise
        self.roughness_scale_accel = roughness_scale_accel
        self.roughness_scale_gyro = roughness_scale_gyro

    def adapt(self, ins: 'StrapdownINS', road_roughness: float, spectral_entropy: float = 0.0) -> None:
        """
        Dynamically scale process noise using environmental roughness.
        
        Args:
            ins: The StepdownINS instance whose covariances will be modified.
            road_roughness: Measured variance of high-frequency kinematic noise.
            spectral_entropy: Entropy of the frequency distribution (0 to 1).
        """
        # Entropy factor: 1.0 (pure tone / localized harmonic) to 2.0 (broadband white noise)
        entropy_factor = 1.0 + spectral_entropy
        
        ins.accel_noise_std = self.base_accel_noise + (road_roughness * self.roughness_scale_accel * entropy_factor)
        ins.gyro_noise_std = self.base_gyro_noise + (road_roughness * self.roughness_scale_gyro * entropy_factor)


class MagneticReliabilityScorer:
    """
    Evaluates the reliability of the local magnetic environment by tracking
    field magnitude stability and proximity to expected Earth field norm.
    """
    def __init__(self, window_size: int = 50, expected_norm: float = 50.0):
        self.expected_norm = expected_norm
        self.window_size = window_size
        self.window = []
        
    def add_sample(self, mag: tuple[float, float, float]) -> float:
        """
        Add a magnetometer sample and return the current reliability score.
        
        Args:
            mag: (x, y, z) magnetic field in uT.
            
        Returns:
            Reliability score in [0.0, 1.0]. 1.0 = perfect matching expected field.
        """
        norm = float(np.linalg.norm(np.array(mag)))
        self.window.append(norm)
        if len(self.window) > self.window_size:
            self.window.pop(0)
            
        if len(self.window) < 5:
            # Low confidence until we gather a minimal window
            return 0.1
            
        mean_norm = float(np.mean(self.window))
        std_norm = float(np.std(self.window))
        
        # Penalize mean deviation from expected norm (e.g. 50uT). 
        # A 10uT deviation represents a significant magnetic anomaly.
        mean_penalty = np.exp(-0.5 * ((mean_norm - self.expected_norm) / 10.0)**2)
        
        # Penalize fluctuations (sudden variations > 5uT mean active distortion)
        variance_penalty = np.exp(-0.5 * (std_norm / 5.0)**2)
        
        return float(mean_penalty * variance_penalty)

