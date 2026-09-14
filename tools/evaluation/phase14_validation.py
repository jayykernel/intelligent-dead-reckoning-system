import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import numpy as np
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.sensors.data_types import ImuSample
from core.calibration.dynamic_error import DynamicErrorAdapter

def run_vibration_test(road_roughness: float, entropy: float) -> dict:
    state = NavState(
        timestamp_ns=0, position_m=(0.0, 0.0, 0.0), velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0), accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )
    ins = StrapdownINS(state, accel_noise_std=0.05, gyro_noise_std=0.005)
    adapter = DynamicErrorAdapter(roughness_scale_accel=0.05, roughness_scale_gyro=0.005)
    
    dt = 0.01
    num_steps = 1000
    time = 0
    
    for i in range(num_steps):
        time += dt
        ts_ns = int(time * 1e9)
        
        # Apply adaptation
        adapter.adapt(ins, road_roughness=road_roughness, spectral_entropy=entropy)
            
        imu = ImuSample(ts_ns, (0.0, 0.0, -9.81), (0.0, 0.0, 0.0))
        ins.propagate(imu, propagate_covariance=True)

    return {
        "accel_noise_std": ins.accel_noise_std,
        "gyro_noise_std": ins.gyro_noise_std,
        "pos_var": ins.covariance[0, 0],
        "vel_var": ins.covariance[3, 3],
        "att_var": ins.covariance[6, 6]
    }

def main():
    print("Phase 14: Dynamic Error Adaptation Validation")
    print(f"{'Condition':<20} | {'Accel Std':<10} | {'Gyro Std':<10} | {'Pos Var (10s)':<15} | {'Vel Var (10s)'}")
    print("-" * 75)
    
    scenarios = [
        ("Perfectly Smooth", 0.0, 0.0),
        ("Normal Road", 1.0, 0.2),
        ("Bumpy/Gravel", 5.0, 0.8),
        ("Severe Vibration", 10.0, 1.0)
    ]
    
    for name, rough, ent in scenarios:
        res = run_vibration_test(rough, ent)
        print(f"{name:<20} | {res['accel_noise_std']:<10.4f} | {res['gyro_noise_std']:<10.4f} | {res['pos_var']:<15.2f} | {res['vel_var']:.2f}")

if __name__ == "__main__":
    main()

from core.calibration.dynamic_error import MagneticReliabilityScorer

def test_magnetic_scorer():
    print("\nPhase 14: Magnetic Reliability Scoring Validation")
    print(f"{'Environment':<25} | {'Mean Norm':<10} | {'Std Dev':<10} | {'Rel Score (0-1)'}")
    print("-" * 65)
    
    # 1. Clean environment
    scorer = MagneticReliabilityScorer(10, expected_norm=50.0)
    for _ in range(10): score = scorer.add_sample((30, 40, 0))
    print(f"{'Clean (50 uT)':<25} | {50.0:<10.1f} | {0.0:<10.1f} | {score:.4f}")
    
    # 2. Hard Iron Distortion (e.g. driving near metal)
    scorer = MagneticReliabilityScorer(10, expected_norm=50.0)
    for _ in range(10): score = scorer.add_sample((50, 60, 0))
    print(f"{'Hard Iron (78 uT)':<25} | {78.1:<10.1f} | {0.0:<10.1f} | {score:.4f}")
    
    # 3. Dynamic Anomaly (e.g. passing a powerline, fluctuating)
    scorer = MagneticReliabilityScorer(10, expected_norm=50.0)
    import random
    random.seed(42)
    for _ in range(10): 
        # Mean near 50 but huge variance
        mag_val = 50.0 + random.uniform(-10, 10)
        score = scorer.add_sample((mag_val, 0, 0))
    print(f"{'Dynamic Fluctuation':<25} | {np.mean(scorer.window):<10.1f} | {np.std(scorer.window):<10.1f} | {score:.4f}")

if __name__ == "__main__":
    test_magnetic_scorer()
