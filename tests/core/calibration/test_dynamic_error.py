import pytest
import numpy as np
from core.calibration.dynamic_error import DynamicErrorAdapter, MagneticReliabilityScorer
from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState

def test_magnetic_reliability_scorer_perfect_field():
    scorer = MagneticReliabilityScorer(window_size=10, expected_norm=50.0)
    
    score = 0.0
    for _ in range(10):
        # Provide exactly 50uT repeatedly 
        score = scorer.add_sample((30.0, 40.0, 0.0))  # norm is 50.0
        
    assert score == 1.0  # Perfect reliability

def test_magnetic_reliability_scorer_distorted_field():
    scorer = MagneticReliabilityScorer(window_size=10, expected_norm=50.0)
    
    score = 0.0
    for _ in range(10):
        # Hard iron distortion pushing norm to e.g., 80uT
        score = scorer.add_sample((80.0, 0.0, 0.0))
        
    # mean_penalty = exp(-0.5 * (30/10)^2) = exp(-4.5) ~ 0.011
    assert score < 0.05
    assert score > 0.0

def test_magnetic_reliability_scorer_fluctuating_field():
    scorer = MagneticReliabilityScorer(window_size=10, expected_norm=50.0)
    
    score = 0.0
    for i in range(10):
        # Add high variance but mean ~ 50
        if i % 2 == 0:
            score = scorer.add_sample((60.0, 0.0, 0.0))
        else:
            score = scorer.add_sample((40.0, 0.0, 0.0))
            
    # var penalty = exp(-0.5 * (10/5)^2) = exp(-2.0) ~ 0.13
    assert score < 0.2

def test_dynamic_error_adapter():
    state = NavState(
        timestamp_ns=0, position_m=(0.0, 0.0, 0.0), velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0), accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )
    ins = StrapdownINS(state, accel_noise_std=0.05, gyro_noise_std=0.005)
    
    adapter = DynamicErrorAdapter(
        base_accel_noise=0.05, 
        base_gyro_noise=0.005,
        roughness_scale_accel=0.1,
        roughness_scale_gyro=0.01
    )
    
    # 0 roughness -> base noise
    adapter.adapt(ins, road_roughness=0.0, spectral_entropy=0.0)
    assert ins.accel_noise_std == 0.05
    assert ins.gyro_noise_std == 0.005
    
    # High roughness -> inflated noise
    adapter.adapt(ins, road_roughness=2.0, spectral_entropy=0.5)
    # entropy_factor = 1.5
    # accel: 0.05 + 2.0 * 0.1 * 1.5 = 0.35
    # gyro: 0.005 + 2.0 * 0.01 * 1.5 = 0.035
    assert abs(ins.accel_noise_std - 0.35) < 1e-6
    assert abs(ins.gyro_noise_std - 0.035) < 1e-6
