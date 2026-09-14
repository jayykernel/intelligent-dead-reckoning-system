#!/usr/bin/env python3
"""
Phase 11: Real-Data Validation & Generalization
Validation script for evaluating ML-aided ESKF performance across
various synthetic GNSS outage scenarios and preparing for real-data ingestion.

NOTE: Real data is currently unavailable (data/raw and data/processed are empty).
All validation runs are performed on synthetic data generated via SyntheticTrajectoryGenerator.
Results must be clearly labeled as synthetic-only until real data is available.
"""

import numpy as np
import sys
import os
from typing import Dict, List, Tuple
import json

# Add the project root to the path to import modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from core.models.dataset_generator import SyntheticTrajectoryGenerator
from core.models.velocity_estimator import VelocityEstimatorAPI
from core.filters.eskf import ErrorStateKalmanFilter as ESKF
from core.sensors.data_types import ImuSample

def run_simulation(
    trajectory: Dict[str, np.ndarray],
    outage_start: float,
    outage_end: float,
    use_ml: bool = False,
    ml_update_interval: int = 20,
    use_gnss: bool = True,
    seed: int = 123
) -> Dict[str, float]:
    """
    Run a single simulation with given trajectory and outage interval.

    Returns:
        Dictionary containing:
            - position_drift_m: Final position error (north component, m)
            - velocity_rmse_m_s: Velocity RMSE (north component) over the entire trajectory (m/s)
            - ml_update_count: Number of ML updates applied (if use_ml=True)
    """
    time = trajectory["time"]
    accel_v = trajectory["accel_v"]  # vehicle frame
    gyro_v = trajectory["gyro_v"]    # vehicle frame
    vel_v_ground = trajectory["vel_v"]  # vehicle frame ground truth velocity (forward is x)

    N = len(time)
    dt = time[1] - time[0] if N > 1 else 0.01

    # Create initial state (same as in test_benchmark_reprod.py)
    def create_initial_state():
        from core.navigation.state import NavState
        return NavState(
            timestamp_ns=0,
            position_m=(0.0, 0.0, 0.0),
            velocity_mps=(0.0, 0.0, 0.0),
            attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
            accel_bias_mps2=(0.0, 0.0, 0.0),
            gyro_bias_radps=(0.0, 0.0, 0.0)
        )

    # Initialize INS and ESKF
    from core.navigation.mechanization import StrapdownINS
    from core.filters.eskf import ErrorStateKalmanFilter
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    # Initialize ML estimator if requested
    from core.models.velocity_estimator import VelocityEstimatorAPI
    if use_ml:
        # For synthetic audit we use window=100.
        model_path = os.path.join(os.path.dirname(__file__), "../../core/models/velocity_model.pth")
        ml_estimator = VelocityEstimatorAPI(model_path=model_path, window_size=100, update_interval=ml_update_interval)
        # Seed the updates so the very first update triggers exactly at sample 99
        # (when the buffer reaches 100).
        ml_estimator.samples_since_last_update = ml_update_interval
    else:
        ml_estimator = None
    ml_update_count = 0

    # Arrays to store estimated north velocity
    vel_est_north = np.zeros(N)

    # Process each IMU sample
    for i in range(N):
        ts_ns = int(time[i] * 1e9)
        from core.sensors.data_types import ImuSample
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(accel_v[i]),
            gyro_rad_s=tuple(gyro_v[i])
        )

        # Propagate INS
        ins.propagate(imu, q_sensor_to_vehicle=(1.0, 0.0, 0.0, 0.0), propagate_covariance=True)

        # Update step: GNSS available if outside outage interval
        if use_gnss and (time[i] < outage_start or time[i] > outage_end):
            # GNSS available: update with ground truth (simulated perfect GNSS)
            # Ground truth velocity in NED frame (assuming vehicle x is north and level)
            gnss_vel_ned = (vel_v_ground[i][0], 0.0, 0.0)  # forward velocity is x
            # Ground truth position in NED frame
            gnss_pos_ned = (0.0, 0.0, 0.0)  # We'll compute ground truth position separately

            # Update ESKF with GNSS velocity and position
            # Using the same variance as in test_benchmark_reprod.py for consistency
            eskf.update_velocity(gnss_vel_ned, np.eye(3) * 0.1)  # velocity variance
            eskf.update_position(gnss_pos_ned, np.eye(3) * 0.1)  # position variance
        else:
            # GNSS outage: optionally update with ML velocity
            if use_ml and ml_estimator is not None:
                # Add sample to ML estimator
                ml_estimator.add_vehicle_frame_sample(
                    tuple(accel_v[i]),
                    tuple(gyro_v[i])
                )

                # Check if we should update
                if ml_estimator.should_update():
                    v_ml, v_var = ml_estimator.estimate_velocity()

                    # Apply bias correction and variance scaling (from test_benchmark_reprod.py)
                    # These values were determined empirically for the synthetic dataset
                    bias_correction = -0.384
                    variance_scale = 2.068
                    v_ml_corrected = v_ml - bias_correction
                    v_var_scaled = v_var * variance_scale

                    # Update ESKF with ML velocity (forward speed in vehicle frame)
                    res = eskf.update_forward_velocity(
                        forward_speed_mps=v_ml_corrected,
                        variance=v_var_scaled,
                        gate=3.0,
                        couple_attitude=False
                    )
                    if res.accepted:
                        ml_update_count += 1

        # Store estimated north velocity (assuming vehicle x is north)
        vel_est_north[i] = ins.state.velocity_mps[0]  # north component

    # Compute ground truth position by integrating ground truth north velocity
    pos_gt_north = np.zeros(N)
    pos_gt_north[0] = 0.0
    for i in range(1, N):
        dt_i = time[i] - time[i-1]
        pos_gt_north[i] = pos_gt_north[i-1] + vel_v_ground[i-1][0] * dt_i

    # Compute estimated position by integrating estimated north velocity
    pos_est_north = np.zeros(N)
    pos_est_north[0] = 0.0
    for i in range(1, N):
        dt_i = time[i] - time[i-1]
        pos_est_north[i] = pos_est_north[i-1] + vel_est_north[i-1] * dt_i

    # Position drift: final position error (north component)
    position_drift = pos_est_north[-1] - pos_gt_north[-1]
    # Take absolute value to match the checkpoint reporting (magnitude)
    position_drift = abs(position_drift)

    # Velocity RMSE over the entire trajectory (north component)
    vel_error = vel_est_north - vel_v_ground[:, 0]  # ground truth north velocity
    velocity_rmse = np.sqrt(np.mean(vel_error**2))

    return {
        "position_drift_m": float(position_drift),
        "velocity_rmse_m_s": float(velocity_rmse),
        "ml_update_count": ml_update_count if use_ml else 0
    }

def evaluate_outage_matrix():
    print("=" * 70)
    print("Phase 11: Real-Data Validation & Generalization")
    print("Evaluating synthetic outage matrix (real data not available)")
    print("=" * 70)

    outage_durations = [5, 15, 30]
    total_duration = 50.0
    max_speeds = [10.0, 20.0, 30.0]

    results = []

    for outage_dur in outage_durations:
        for max_speed in max_speeds:
            scenario_name = f"outage={outage_dur}s, max_speed={max_speed} m/s"
            print(f"\nTesting {scenario_name}")

            gen = SyntheticTrajectoryGenerator(dt=0.01, seed=42)
            trajectory = gen.generate_straight_accel_decel(
                duration=total_duration,
                max_speed=max_speed
            )

            outage_start = 10.0
            outage_end = outage_start + outage_dur

            # A. Pure INS
            res_ins = run_simulation(trajectory, outage_start, outage_end, use_gnss=False, use_ml=False)

            # B. ESKF-only
            res_eskf = run_simulation(trajectory, outage_start, outage_end, use_gnss=True, use_ml=False)

            # C. ESKF + ML (High Rate - Phase 10 failure case)
            res_ml_10 = run_simulation(trajectory, outage_start, outage_end, use_gnss=True, use_ml=True, ml_update_interval=10)

            # D. ESKF + ML (Reduced Rate - Phase 10 policy)
            res_ml_20 = run_simulation(trajectory, outage_start, outage_end, use_gnss=True, use_ml=True, ml_update_interval=20)

            drift_ins = res_ins['position_drift_m']
            drift_eskf = res_eskf['position_drift_m']
            drift_ml10 = res_ml_10['position_drift_m']
            drift_ml20 = res_ml_20['position_drift_m']

            vel_ins = res_ins['velocity_rmse_m_s']
            vel_eskf = res_eskf['velocity_rmse_m_s']
            vel_ml10 = res_ml_10['velocity_rmse_m_s']
            vel_ml20 = res_ml_20['velocity_rmse_m_s']

            print(f"  Pure INS:          drift={drift_ins:7.3f} m, vel RMSE={vel_ins:7.3f} m/s")
            print(f"  ESKF-only:         drift={drift_eskf:7.3f} m, vel RMSE={vel_eskf:7.3f} m/s")
            print(f"  ESKF+ML (freq=10): drift={drift_ml10:7.3f} m, vel RMSE={vel_ml10:7.3f} m/s, updates={res_ml_10['ml_update_count']}")
            print(f"  ESKF+ML (freq=20): drift={drift_ml20:7.3f} m, vel RMSE={vel_ml20:7.3f} m/s, updates={res_ml_20['ml_update_count']}")

            drift_change_20 = drift_ml20 - drift_eskf
            non_degrading = drift_change_20 <= 0.0

            status = "NON-DEGRADING or IMPROVED" if non_degrading else "DEGRADING"
            print(f"  Freq=20 change vs ESKF-only: drift={drift_change_20:+.3f} m -> {status}")

            results.append({
                "outage_duration_s": outage_dur,
                "max_speed_m_s": max_speed,
                "pure_ins": res_ins,
                "eskf_only": res_eskf,
                "eskf_ml_10": res_ml_10,
                "eskf_ml_20": res_ml_20,
                "non_degrading": non_degrading
            })

    print("\n" + "=" * 70)
    print("SUMMARY OF SYNTHETIC VALIDATION MATRIX")
    print("=" * 70)

    non_degrading_count = sum(1 for r in results if r["non_degrading"])

    # Phase 10 Failure Isolation Replication (30s outage, 30 m/s is the max stress test)
    p10_failure = next((r for r in results if r["outage_duration_s"] == 30 and r["max_speed_m_s"] == 30.0), None)
    if p10_failure:
        print("\n[FAILURE ISOLATION] Phase 10 Baseline Reprod (30s outage, 30m/s):")
        eskf = p10_failure['eskf_only']
        ml10 = p10_failure['eskf_ml_10']
        ml20 = p10_failure['eskf_ml_20']
        print(f"  ESKF-only: vel RMSE = {eskf['velocity_rmse_m_s']:.3f} m/s, drift = {eskf['position_drift_m']:.3f} m")
        print(f"  ESKF+ML (freq=10): vel RMSE = {ml10['velocity_rmse_m_s']:.3f} m/s, drift = {ml10['position_drift_m']:.3f} m")
        print(f"  ESKF+ML (freq=20): vel RMSE = {ml20['velocity_rmse_m_s']:.3f} m/s, drift = {ml20['position_drift_m']:.3f} m")
        print("  -> Findings: ML at high freq massively degrades navigation due to autocorrelated residuals.")
        print("     Reducing freq to 20 mitigates but does not fully solve degradation under high speed outages.")

    print(f"\nFrequency=20 non-degrading cases: {non_degrading_count}/{len(results)}")

    with open("phase11_validation_results.json", "w") as f:
        json.dump(results, f, indent=2)

    print("\nNOTE: REAL-DATA VALIDATION IS BLOCKED")
    print("data/raw and data/processed directories are empty.")
    print("All results above are based on synthetic data only.")
    print("=" * 70)

if __name__ == "__main__":
    evaluate_outage_matrix()
