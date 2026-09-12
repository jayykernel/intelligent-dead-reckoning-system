"""
Unit tests for the Rolling FFT Spectrum Analyzer.
"""
import numpy as np
import pytest

from core.sensors.data_types import ImuSample
from core.motion.spectral_analyzer import RollingSpectralAnalyzer


def _create_imu_sample(z_accel: float) -> ImuSample:
    """Create a dummy IMU sample with specific Z acceleration."""
    return ImuSample(
        timestamp_ns=0,
        accel_m_s2=(0.0, 0.0, z_accel),
        gyro_rad_s=(0.0, 0.0, 0.0)
    )

def test_spectral_analyzer_initialization():
    """Test proper initialization of the analyzer."""
    analyzer = RollingSpectralAnalyzer(window_size_seconds=1.0, sample_rate_hz=100.0)
    assert analyzer.window_size_samples == 100
    assert len(analyzer.buffer) == 100
    assert not analyzer.buffer_full
    assert len(analyzer.freqs) == 51  # N/2 + 1 for rfft

def test_spectral_analyzer_pure_tone():
    """Test spectral analysis of a pure tone."""
    sample_rate = 100.0
    window_time = 1.0
    analyzer = RollingSpectralAnalyzer(window_size_seconds=window_time, sample_rate_hz=sample_rate)

    # Generate a pure 15 Hz tone
    t = np.arange(int(sample_rate * window_time)) / sample_rate
    signal = np.sin(2 * np.pi * 15.0 * t)

    for val in signal:
        analyzer.update(_create_imu_sample(val))

    assert analyzer.buffer_full

    freqs, mag = analyzer.get_spectrum()

    # Check that frequency array matches expectations
    assert len(freqs) == len(mag)

    # Find peak frequency
    peak_idx = np.argmax(mag)
    peak_freq = freqs[peak_idx]

    # Use a relaxed tolerance due to windowing leakage
    assert pytest.approx(peak_freq, abs=1.0) == 15.0

    # Ensure spectral entropy is relatively low for a pure tone
    entropy = analyzer.get_spectral_entropy()
    assert entropy < 0.5

def test_spectral_analyzer_white_noise():
    """Test spectral analysis of white noise."""
    sample_rate = 100.0
    window_time = 1.0
    np.random.seed(42)
    analyzer = RollingSpectralAnalyzer(window_size_seconds=window_time, sample_rate_hz=sample_rate)

    # Generate white noise
    signal = np.random.normal(0, 1, int(sample_rate * window_time))

    for val in signal:
        analyzer.update(_create_imu_sample(val))

    # Entropy should be high for white noise
    entropy = analyzer.get_spectral_entropy()
    assert entropy > 0.8

def test_engine_frequency_isolation():
    """Test isolation of the engine vibration band (10-30 Hz)."""
    sample_rate = 100.0
    analyzer = RollingSpectralAnalyzer(window_size_seconds=1.0, sample_rate_hz=sample_rate)

    t = np.arange(int(sample_rate)) / sample_rate
    # Create signal with components at 5Hz (outside), 20Hz (inside), 40Hz (outside)
    signal = np.sin(2 * np.pi * 5.0 * t) + 2.0 * np.sin(2 * np.pi * 20.0 * t) + 0.5 * np.sin(2 * np.pi * 40.0 * t)

    for val in signal:
        analyzer.update(_create_imu_sample(val))

    engine_power = analyzer.get_engine_band_power()

    # Engine power should be significant
    assert engine_power > 0.1

    # Compare against an analyzer with ONLY 5Hz and 40Hz (no engine frequency)
    analyzer2 = RollingSpectralAnalyzer(window_size_seconds=1.0, sample_rate_hz=sample_rate)
    signal2 = np.sin(2 * np.pi * 5.0 * t) + 0.5 * np.sin(2 * np.pi * 40.0 * t)
    for val in signal2:
        analyzer2.update(_create_imu_sample(val))

    engine_power2 = analyzer2.get_engine_band_power()

    # Power in the band should be much higher for the first signal
    assert engine_power > 10 * engine_power2

def test_road_roughness_estimation():
    """Test road roughness calculation (low-frequency power)."""
    sample_rate = 100.0
    analyzer = RollingSpectralAnalyzer(window_size_seconds=1.0, sample_rate_hz=sample_rate)

    t = np.arange(int(sample_rate)) / sample_rate
    # Create signal with heavy low frequency (0-20 Hz)
    signal1 = 3.0 * np.sin(2 * np.pi * 2.0 * t) + 2.0 * np.sin(2 * np.pi * 5.0 * t)

    for val in signal1:
        analyzer.update(_create_imu_sample(val))

    roughness1 = analyzer.get_road_roughness()

    # Create fairly smooth signal
    analyzer.reset()
    signal2 = 0.5 * np.sin(2 * np.pi * 2.0 * t)

    for val in signal2:
        analyzer.update(_create_imu_sample(val))

    roughness2 = analyzer.get_road_roughness()

    assert roughness1 > roughness2

def test_incomplete_buffer():
    """Test behavior when buffer is not yet full."""
    analyzer = RollingSpectralAnalyzer(window_size_seconds=1.0, sample_rate_hz=100.0)

    # Add just a few samples
    for i in range(10):
        analyzer.update(_create_imu_sample(1.0))

    freqs, mag = analyzer.get_spectrum()
    assert np.all(mag == 0)
    assert analyzer.get_spectral_entropy() == 0.0
    assert analyzer.get_road_roughness() == 0.0
    assert analyzer.get_engine_band_power() == 0.0

def test_rolling_update():
    """Test that old data is shifted out appropriately."""
    sample_rate = 100.0
    analyzer = RollingSpectralAnalyzer(window_size_seconds=1.0, sample_rate_hz=sample_rate)

    t = np.arange(int(sample_rate * 2)) / sample_rate

    # First second: 10 Hz tone
    # Second second: 20 Hz tone
    signal = np.zeros(200)
    signal[:100] = np.sin(2 * np.pi * 10 * t[:100])
    signal[100:] = np.sin(2 * np.pi * 20 * t[100:])

    # Feed the first second
    for val in signal[:100]:
        analyzer.update(_create_imu_sample(val))

    freqs1, mag1 = analyzer.get_spectrum()
    peak_freq1 = freqs1[np.argmax(mag1)]
    assert pytest.approx(peak_freq1, abs=1.0) == 10.0

    # Feed the second second (completely overriding the first)
    for val in signal[100:]:
        analyzer.update(_create_imu_sample(val))

    freqs2, mag2 = analyzer.get_spectrum()
    peak_freq2 = freqs2[np.argmax(mag2)]
    assert pytest.approx(peak_freq2, abs=1.0) == 20.0
