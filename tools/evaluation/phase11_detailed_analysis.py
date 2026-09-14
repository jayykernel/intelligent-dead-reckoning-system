#!/usr/bin/env python3
"""
Phase 11: Detailed Failure Analysis
Reproduces all 9 scenarios with comprehensive metrics to diagnose ML generalization failure.
"""

import numpy as np
import sys
import os
from typing import Dict, List, Tuple
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from core.models.dataset_generator import SyntheticTrajectoryGenerator
from core.models.velocity_estimator import VelocityEstimatorAPI
from core.filters.eskf import ErrorStateKalmanFilter
from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState
from core.sensors.data_types import ImuSample

def create_initial_state():
    """Create initial navigation state."""
    return NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )

def run_detailed_scenario(
    trajectory: Dict[str, np.ndarray],
    outage_start: float,
    outage_end: float,
    use_ml: bool = False,
    ml_update_interval: int = 20,
    bias_correction: float = -0.384,
    variance_scale: float = 2.068,
    seed: int = 42
) -> Dict:
    """
    Run a single scenario with detailed ML metrics collection.

    Returns detailed metrics including:
    - ML predictions (raw and corrected)
    - ML uncertainties
    - ML residuals
    - Update acceptance status
    - Innovation magnitudes
    """
    time = trajectory["time"]
    accel_v = trajectory["accel_v"]
    gyro_v = trajectory["gyro_v"]
    vel_v_ground = trajectory["vel_v"]

    N = len(time)
    dt = time[1] - time[0] if N > 1 else 0.01

    # Initialize INS and ESKF
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    # Initialize ML estimator
    ml_estimator = None
    if use_ml:
        model_path = os.path.join(os.path.dirname(__file__), "../../core/models/velocity_model.pth")
        ml_estimator = VelocityEstimatorAPI(model_path=model_path, window_size=100, update_interval=ml_update_interval)
        ml_estimator.samples_since_last_update = ml_update_interval

    # Storage for detailed metrics
    vel_est_north = np.zeros(N)
    ml_predictions = []
    ml_uncertainties = []
    ml_residuals = []
    ml_update_times = []
    ml_acceptance_status = []
    ml_innovations = []

    ml_update_count = 0

    # Process each sample
    for i in range(N):
        ts_ns = int(time[i] * 1e9)
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(accel_v[i]),
            gyro_rad_s=tuple(gyro_v[i])
        )

        ins.propagate(imu, q_sensor_to_vehicle=(1.0, 0.0, 0.0, 0.0), propagate_covariance=True)

        # GNSS updates outside outage
        if time[i] < outage_start or time[i] > outage_end:
            gnss_vel_ned = (vel_v_ground[i][0], 0.0, 0.0)
            gnss_pos_ned = (0.0, 0.0, 0.0)
            eskf.update_velocity(gnss_vel_ned, np.eye(3) * 0.1)
            eskf.update_position(gnss_pos_ned, np.eye(3) * 0.1)
        else:
            # GNSS outage: try ML update
            if use_ml and ml_estimator is not None:
                ml_estimator.add_vehicle_frame_sample(
                    tuple(accel_v[i]),
                    tuple(gyro_v[i])
                )

                if ml_estimator.should_update():
                    v_ml_raw, v_var_raw = ml_estimator.estimate_velocity()
                    v_ml_corrected = v_ml_raw - bias_correction
                    v_var_scaled = v_var_raw * variance_scale

                    # Ground truth for residual calculation
                    v_gt = vel_v_ground[i][0]
                    residual_raw = v_ml_raw - v_gt
                    residual_corrected = v_ml_corrected - v_gt

                    # Store ML metrics
                    ml_predictions.append({
                        'time': time[i],
                        'raw': v_ml_raw,
                        'corrected': v_ml_corrected,
                        'ground_truth': v_gt
                    })
                    ml_uncertainties.append({
                        'time': time[i],
                        'raw_variance': v_var_raw,
                        'scaled_variance': v_var_scaled,
                        'std_dev': np.sqrt(v_var_scaled)
                    })
                    ml_residuals.append({
                        'time': time[i],
                        'raw': residual_raw,
                        'corrected': residual_corrected,
                        'abs_corrected': abs(residual_corrected)
                    })
                    ml_update_times.append(time[i])

                    # Apply update to ESKF
                    res = eskf.update_forward_velocity(
                        forward_speed_mps=v_ml_corrected,
                        variance=v_var_scaled,
                        gate=3.0,
                        couple_attitude=False
                    )

                    ml_acceptance_status.append({
                        'time': time[i],
                        'accepted': res.accepted,
                        'mahalanobis': res.mahalanobis_dist,
                        'innovation': float(res.innovation[0]) if res.accepted else None
                    })

                    if res.accepted:
                        ml_update_count += 1
                        if res.innovation is not None:
                            ml_innovations.append({
                                'time': time[i],
                                'magnitude': float(res.innovation[0])
                            })

        vel_est_north[i] = ins.state.velocity_mps[0]

    # Compute position drift
    pos_gt_north = np.zeros(N)
    pos_est_north = np.zeros(N)
    for i in range(1, N):
        dt_i = time[i] - time[i-1]
        pos_gt_north[i] = pos_gt_north[i-1] + vel_v_ground[i-1][0] * dt_i
        pos_est_north[i] = pos_est_north[i-1] + vel_est_north[i-1] * dt_i

    position_drift = abs(pos_est_north[-1] - pos_gt_north[-1])

    # Velocity RMSE
    vel_error = vel_est_north - vel_v_ground[:, 0]
    velocity_rmse = np.sqrt(np.mean(vel_error**2))
    max_vel_error = np.max(np.abs(vel_error))

    # ML prediction statistics
    ml_stats = {}
    if ml_predictions:
        ml_pred_array = np.array([p['corrected'] for p in ml_predictions])
        ml_gt_array = np.array([p['ground_truth'] for p in ml_predictions])
        ml_res_array = ml_pred_array - ml_gt_array

        ml_stats = {
            'mean_prediction': float(np.mean(ml_pred_array)),
            'mean_ground_truth': float(np.mean(ml_gt_array)),
            'mean_residual': float(np.mean(ml_res_array)),
            'std_residual': float(np.std(ml_res_array)),
            'rmse': float(np.sqrt(np.mean(ml_res_array**2))),
            'max_abs_residual': float(np.max(np.abs(ml_res_array))),
            'mean_uncertainty_std': float(np.mean([u['std_dev'] for u in ml_uncertainties])),
            'acceptance_rate': float(sum(1 for s in ml_acceptance_status if s['accepted']) / len(ml_acceptance_status)) if ml_acceptance_status else 0.0
        }

        # Compute residual autocorrelation at lag 1
        if len(ml_res_array) > 1:
            ml_stats['residual_autocorr_lag1'] = float(np.corrcoef(ml_res_array[:-1], ml_res_array[1:])[0, 1])

    return {
        'position_drift_m': position_drift,
        'velocity_rmse_m_s': velocity_rmse,
        'max_velocity_error_m_s': max_vel_error,
        'ml_update_count': ml_update_count,
        'ml_predictions': ml_predictions,
        'ml_uncertainties': ml_uncertainties,
        'ml_residuals': ml_residuals,
        'ml_acceptance_status': ml_acceptance_status,
        'ml_innovations': ml_innovations,
        'ml_statistics': ml_stats,
        'distance_travelled_m': float(pos_gt_north[-1])
    }

def main():
    """Run detailed analysis on all 9 scenarios."""
    print("=" * 80)
    print("PHASE 11 DETAILED FAILURE ANALYSIS")
    print("=" * 80)

    outage_durations = [5, 15, 30]
    max_speeds = [10.0, 20.0, 30.0]
    total_duration = 50.0
    outage_start = 10.0

    results = []

    for outage_dur in outage_durations:
        for max_speed in max_speeds:
            print(f"\n{'='*80}")
            print(f"Scenario: {outage_dur}s outage, {max_speed} m/s max speed")
            print(f"{'='*80}")

            # Generate trajectory
            gen = SyntheticTrajectoryGenerator(dt=0.01, seed=42)
            trajectory = gen.generate_straight_accel_decel(
                duration=total_duration,
                max_speed=max_speed
            )

            outage_end = outage_start + outage_dur

            # Run ESKF-only
            print("\nRunning ESKF-only...")
            result_eskf = run_detailed_scenario(
                trajectory, outage_start, outage_end,
                use_ml=False, ml_update_interval=20, seed=42
            )

            # Run ESKF+ML
            print("Running ESKF+ML (update_interval=20)...")
            result_ml = run_detailed_scenario(
                trajectory, outage_start, outage_end,
                use_ml=True, ml_update_interval=20, seed=42
            )

            # Compute degradation
            drift_change = result_ml['position_drift_m'] - result_eskf['position_drift_m']
            vel_rmse_change = result_ml['velocity_rmse_m_s'] - result_eskf['velocity_rmse_m_s']
            non_degrading = (drift_change <= 0.0) and (vel_rmse_change <= 0.0)

            # Print summary
            print(f"\nResults:")
            print(f"  ESKF-only:")
            print(f"    Position drift: {result_eskf['position_drift_m']:.3f} m")
            print(f"    Velocity RMSE:  {result_eskf['velocity_rmse_m_s']:.3f} m/s")
            print(f"    Max vel error:  {result_eskf['max_velocity_error_m_s']:.3f} m/s")

            print(f"  ESKF+ML:")
            print(f"    Position drift: {result_ml['position_drift_m']:.3f} m")
            print(f"    Velocity RMSE:  {result_ml['velocity_rmse_m_s']:.3f} m/s")
            print(f"    Max vel error:  {result_ml['max_velocity_error_m_s']:.3f} m/s")
            print(f"    ML updates:     {result_ml['ml_update_count']}")

            if result_ml['ml_statistics']:
                stats = result_ml['ml_statistics']
                print(f"  ML Statistics:")
                print(f"    Mean prediction:    {stats['mean_prediction']:.3f} m/s")
                print(f"    Mean ground truth:  {stats['mean_ground_truth']:.3f} m/s")
                print(f"    Mean residual:      {stats['mean_residual']:.3f} m/s")
                print(f"    Residual RMSE:      {stats['rmse']:.3f} m/s")
                print(f"    Max abs residual:   {stats['max_abs_residual']:.3f} m/s")
                print(f"    Mean uncertainty:   {stats['mean_uncertainty_std']:.3f} m/s")
                print(f"    Acceptance rate:    {stats['acceptance_rate']*100:.1f}%")
                if 'residual_autocorr_lag1' in stats:
                    print(f"    Autocorr (lag-1):   {stats['residual_autocorr_lag1']:.3f}")

            print(f"  Change vs ESKF-only:")
            print(f"    Drift change:       {drift_change:+.3f} m")
            print(f"    Vel RMSE change:    {vel_rmse_change:+.3f} m/s")
            print(f"    Status:             {'[OK] NON-DEGRADING' if non_degrading else '[FAIL] DEGRADING'}")

            # Store results
            result_record = {
                'outage_duration_s': outage_dur,
                'max_speed_m_s': max_speed,
                'distance_travelled_m': result_eskf['distance_travelled_m'],
                'eskf_only': {
                    'position_drift_m': result_eskf['position_drift_m'],
                    'velocity_rmse_m_s': result_eskf['velocity_rmse_m_s'],
                    'max_velocity_error_m_s': result_eskf['max_velocity_error_m_s']
                },
                'eskf_ml': {
                    'position_drift_m': result_ml['position_drift_m'],
                    'velocity_rmse_m_s': result_ml['velocity_rmse_m_s'],
                    'max_velocity_error_m_s': result_ml['max_velocity_error_m_s'],
                    'ml_update_count': result_ml['ml_update_count'],
                    'ml_statistics': result_ml['ml_statistics']
                },
                'change_vs_eskf_only': {
                    'position_drift_m': drift_change,
                    'velocity_rmse_m_s': vel_rmse_change
                },
                'non_degrading': bool(non_degrading)
            }
            results.append(result_record)

    # Save detailed results
    output_file = "phase11_detailed_results.json"
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n{'='*80}")
    print(f"Detailed results saved to {output_file}")
    print(f"{'='*80}")

    return results

if __name__ == "__main__":
    main()
