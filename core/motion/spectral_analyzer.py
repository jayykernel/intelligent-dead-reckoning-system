"""
Rolling FFT spectrum analyzer for vehicle motion characterization.
Provides spectral entropy, road roughness estimation, and engine frequency band isolation.
"""
import numpy as np
from typing import Tuple, Optional
from core.sensors.data_types import ImuSample


class RollingSpectralAnalyzer:
    """
    Rolling FFT spectrum analyzer for analyzing vibration signals.
    Maintains a buffer of recent samples and computes the FFT on demand.
    """

    def __init__(self, window_size_seconds: float, sample_rate_hz: float):
        """
        Initialize the spectral analyzer.

        Args:
            window_size_seconds: Duration of the rolling window in seconds.
            sample_rate_hz: Sample rate of the input signal in Hz.
        """
        self.window_size_seconds = window_size_seconds
        self.sample_rate_hz = sample_rate_hz
        self.window_size_samples = int(window_size_seconds * sample_rate_hz)

        # Buffer for the signal (we'll use vertical acceleration for road roughness)
        self.buffer = np.zeros(self.window_size_samples)
        self.buffer_index = 0
        self.buffer_full = False

        # Precompute frequency bins for positive frequencies
        self.freqs = np.fft.rfftfreq(self.window_size_samples, 1.0 / sample_rate_hz)

        # Cache for the latest spectrum magnitude
        self._magnitude_spectrum = None
        self._power_spectrum = None

    def update(self, imu_sample: ImuSample) -> None:
        """
        Update the analyzer with a new IMU sample.
        Uses the vertical acceleration (z-axis) for spectral analysis.

        Args:
            imu_sample: New IMU sample.
        """
        # Extract vertical acceleration (assuming IMU frame is aligned with vehicle frame)
        # Z-axis is positive downward in NED, but for vibration analysis we use magnitude
        vertical_acc = imu_sample.accel_m_s2[2]  # Z-axis

        # Insert sample into buffer
        self.buffer[self.buffer_index] = vertical_acc
        self.buffer_index += 1

        if self.buffer_index >= self.window_size_samples:
            self.buffer_index = 0
            self.buffer_full = True

        # Invalidate cached spectrum
        self._magnitude_spectrum = None
        self._power_spectrum = None

    def _compute_spectrum(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the FFT magnitude and power spectrum of the current buffer.
        Returns:
            Tuple of (frequencies, magnitude_spectrum)
        """
        if not self.buffer_full:
            # Not enough data yet
            self._magnitude_spectrum = np.zeros_like(self.freqs)
            self._power_spectrum = np.zeros_like(self.freqs)
            return self.freqs, self._magnitude_spectrum

        # Apply a window function to reduce spectral leakage
        windowed_signal = self.buffer * np.hanning(self.window_size_samples)

        # Compute FFT
        fft_result = np.fft.rfft(windowed_signal)
        magnitude = np.abs(fft_result) / (self.window_size_samples / 2.0)
        power = magnitude ** 2

        self._magnitude_spectrum = magnitude
        self._power_spectrum = power
        return self.freqs, magnitude

    def get_spectrum(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the current magnitude spectrum.
        Returns:
            Tuple of (frequencies in Hz, magnitude spectrum).
        """
        if self._magnitude_spectrum is None:
            return self._compute_spectrum()
        return self.freqs, self._magnitude_spectrum

    def get_power_spectrum(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the current power spectrum.
        Returns:
            Tuple of (frequencies in Hz, power spectrum).
        """
        if self._power_spectrum is None:
            self._compute_spectrum()
        return self.freqs, self._power_spectrum

    def get_spectral_entropy(self) -> float:
        """
        Compute the spectral entropy of the current spectrum.
        Spectral entropy measures the unpredictability or flatness of the spectrum.
        Returns:
            Spectral entropy value (0 = pure tone, 1 = white noise).
        """
        if not self.buffer_full:
            return 0.0
        freqs, magnitude = self.get_spectrum()
        # Avoid log(0) by adding a small epsilon
        eps = 1e-10
        power = magnitude ** 2 + eps
        # Normalize to a probability distribution
        power_norm = power / np.sum(power)
        # Compute Shannon entropy
        entropy = -np.sum(power_norm * np.log2(power_norm))
        # Normalize by the maximum possible entropy (log2(N))
        max_entropy = np.log2(len(power_norm))
        return entropy / max_entropy if max_entropy > 0 else 0.0

    def get_road_roughness(self) -> float:
        """
        Estimate road roughness from the power spectrum.
        Uses a weighted integral of the power spectrum, emphasizing frequencies
        relevant to vehicle suspension (typically 0-50 Hz for road roughness).
        Returns:
            Road roughness index (higher = rougher road).
        """
        freqs, power = self.get_power_spectrum()
        # Weighting function: emphasize lower frequencies (road roughness)
        # Simple approach: use power in 0-20 Hz band (can be adjusted)
        mask = (freqs >= 0.0) & (freqs <= 20.0)
        if not np.any(mask):
            return 0.0
        # Integral of power over the band (approximated by sum)
        if len(power[mask]) > 1:
            roughness = np.trapezoid(power[mask], freqs[mask])
        else:
            roughness = 0.0
        return roughness

    def get_engine_band_power(self) -> float:
        """
        Get the power in the engine frequency band (10-30 Hz).
        Returns:
            Power in the 10-30 Hz band.
        """
        freqs, power = self.get_power_spectrum()
        mask = (freqs >= 10.0) & (freqs <= 30.0)
        if not np.any(mask):
            return 0.0
        if len(power[mask]) > 1:
            return np.trapezoid(power[mask], freqs[mask])
        else:
            return 0.0

    def reset(self) -> None:
        """Reset the analyzer to initial state."""
        self.buffer = np.zeros(self.window_size_samples)
        self.buffer_index = 0
        self.buffer_full = False
        self._magnitude_spectrum = None
        self._power_spectrum = None