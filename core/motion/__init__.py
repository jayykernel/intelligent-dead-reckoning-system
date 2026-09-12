"""Motion intelligence modules: ZUPT, vibration analysis, motion classification."""

from core.motion.zupt_detector import ZuptDetector
from core.motion.spectral_analyzer import RollingSpectralAnalyzer

__all__ = ['ZuptDetector', 'RollingSpectralAnalyzer']
