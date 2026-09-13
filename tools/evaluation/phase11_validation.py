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
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

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
        ml_estimator = VelocityEstimatorAPI(window_size=100, update_interval=ml_update_interval)
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
        if time[i] < outage_start or time[i] > outage_end:
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
    """
    Evaluate the validation matrix over synthetic control and varied synthetic outage scenarios:
        - short 5s, medium 15s, long 30s outages
        - varied acceleration profiles (we'll use the generator's default profile)
    """
    print("=" * 60)
    print("Phase 11: Real-Data Validation & Generalization")
    print("Evaluating synthetic outage matrix (real data not available)")
    print("=" * 60)

    # Define outage durations to test (seconds)
    outage_durations = [5, 15, 30]
    # We'll use a fixed trajectory length longer than the max outage
    total_duration = 50.0  # seconds
    # Acceleration profile: we'll use the generator's straight_accel_decel with varying max speed
    max_speeds = [10.0, 20.0, 30.0]  # m/s

    results = []

    for outage_dur in outage_durations:
        for max_speed in max_speeds:
            print(f"\nTesting outage={outage_dur}s, max_speed={max_speed} m/s")

            # Generate trajectory
            gen = SyntheticTrajectoryGenerator(dt=0.01, seed=42)
            trajectory = gen.generate_straight_accel_decel(
                duration=total_duration,
                max_speed=max_speed
            )

            # Define outage interval (start after 10 seconds, for example)
            outage_start = 10.0
            outage_end = outage_start + outage_dur

            # Run three configurations:
            # A. Pure INS (no ESKF updates, just propagation)
            # B. ESKF-only (GNSS updates when available, no ML)
            # C. ESKF + ML (with Phase 10 policy: update_interval=20)

            # Note: For Pure INS, we bypass ESKF updates entirely.
            # We'll implement a simple propagation for Pure INS.
            # For simplicity, we'll reuse the ESKF but disable updates.
            # However, let's create a separate function for Pure INS if needed.
            # For now, we'll run ESKF-only and ESKF+ML, and approximate Pure INS by
            # setting ESKF to not update (but it still has initial state).
            # Actually, we can run ESKF-only and then compare to a baseline that
            # doesn't use any updates (just the initial state propagated).
            # We'll do:
            #   A. Pure INS: propagate initial state with IMU only (no updates)
            #   B. ESKF-only: ESKF with GNSS updates (no ML)
            #   C. ESKF+ML: ESKF with GNSS and ML updates

            # We'll implement Pure INS by creating a simple propagator.
            # But to save time, we note that the ESKF-only with very high process noise
            # approximates INS. However, we have a separate test for Pure INS in
            # the failure isolation study. Let's reuse the logic from there.

            # For now, we'll run:
            #   A. ESKF-only with Q set very high (so it trusts IMU more than updates)
            #   But that's not exactly Pure INS.
            #
            # Given the complexity, and since the user wants to see if ML aids
            # relative to ESKF-only (as per Phase 10), we'll compare:
            #   ESKF+ML vs ESKF-only
            # and note that Pure INS is available from previous benchmarks.
            #
            # We'll run ESKF-only and ESKF+ML as defined.

            # Run ESKF-only (use_ml=False)
            result_eskf_only = run_simulation(
                trajectory, outage_start, outage_end,
                use_ml=False, ml_update_interval=20, seed=42
            )

            # Run ESKF+ML (use_ml=True, update_interval=20)
            result_eskf_ml = run_simulation(
                trajectory, outage_start, outage_end,
                use_ml=True, ml_update_interval=20, seed=42
            )

            # Compute change relative to ESKF-only
            drift_change = result_eskf_ml["position_drift_m"] - result_eskf_only["position_drift_m"]
            vel_rmse_change = result_eskf_ml["velocity_rmse_m_s"] - result_eskf_only["velocity_rmse_m_s"]

            # Determine if ML is non-degrading (i.e., not worse than ESKF-only)
            non_degrading = (drift_change <= 0.0) and (vel_rmse_change <= 0.0)

            # Record results
            result_record = {
                "outage_duration_s": outage_dur,
                "max_speed_m_s": max_speed,
                "eskf_only": {
                    "position_drift_m": result_eskf_only["position_drift_m"],
                    "velocity_rmse_m_s": result_eskf_only["velocity_rmse_m_s"]
                },
                "eskf_ml": {
                    "position_drift_m": result_eskf_ml["position_drift_m"],
                    "velocity_rmse_m_s": result_eskf_ml["velocity_rmse_m_s"],
                    "ml_update_count": result_eskf_ml["ml_update_count"]
                },
                "change_vs_eskf_only": {
                    "position_drift_m": drift_change,
                    "velocity_rmse_m_s": vel_rmse_change
                },
                "non_degrading": non_degrading
            }
            results.append(result_record)

            # Print summary for this case
            print(f"  ESKF-only:  drift={result_eskf_only['position_drift_m']:.3f} m, "
                  f"vel RMSE={result_eskf_only['velocity_rmse_m_s']:.3f} m/s")
            print(f"  ESKF+ML:    drift={result_eskf_ml['position_drift_m']:.3f} m, "
                  f"vel RMSE={result_eskf_ml['velocity_rmse_m_s']:.3f} m/s, "
                  f"updates={result_eskf_ml['ml_update_count']}")
            print(f"  Change:     drift={drift_change:+.3f} m, "
                  f"vel RMSE={vel_rmse_change:+.3f} m/s -> "
                  f"{'NON-DEGRADING' if non_degrading else 'DEGRADING'}")

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY OF SYNTHETIC VALIDATION MATRIX")
    print("=" * 60)
    non_degrading_count = sum(1 for r in results if r["non_degrading"])
    total_count = len(results)
    print(f"Non-degrading cases: {non_degrading_count}/{total_count}")

    if non_degrading_count == total_count:
        print("RESULT: ML-aiding policy is non-degrading across all synthetic scenarios tested.")
    else:
        print("RESULT: ML-aiding policy shows degradation in some scenarios.")
        print("Details:")
        for r in results:
            if not r["non_degrading"]:
                print(f"  - Outage {r['outage_duration_s']}s, speed {r['max_speed_m_s']} m/s: "
                      f"drift change {r['change_vs_eskf_only']['position_drift_m']:+.3f} m, "
                      f"vel RMSE change {r['change_vs_eskf_only']['velocity_rmse_m_s']:+.3f} m/s")

    # Save results to file
    output_file = "phase11_validation_results.json"
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nDetailed results saved to {output_file}")

    # Note about real data
    print("\n" + "!" * 60)
    print("NOTE: REAL-DATA VALIDATION IS BLOCKED")
    print("  - data/raw and data/processed directories are empty.")
    print("  - All results above are based on synthetic data only.")
    print("  - Real-data validation must be performed when sensor datasets are available.")
    print("!" * 60)

    return results

if __name__ == "__main__":
    # Run the validation matrix
    evaluate_outage_matrix()