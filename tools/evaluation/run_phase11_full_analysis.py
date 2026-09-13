#!/usr/bin/env python3
"""
Comprehensive Phase 11 Generalization Analysis with trained model.
Runs all 9 scenarios with trained model, evaluating:
1. Model loading & prediction accuracy (observability, bias, RMSE, uncertainty, autocorrelation)
2. ESKF-only vs ESKF+ML navigation metrics (position drift, velocity RMSE, max error, update count)
3. Update policy comparison (interval 20, 30, 40, and adaptive)
4. Training vs Evaluation distribution comparison
"""

import os
import sys
import json
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))

from core.models.dataset_generator import SyntheticTrajectoryGenerator
from core.models.velocity_estimator import VelocityEstimatorAPI, Velocity1DCNN
from core.filters.eskf import ErrorStateKalmanFilter
from core.navigation.mechanization import StrapdownINS
from core.navigation.state import NavState
from core.sensors.data_types import ImuSample

PROJECT_ROOT = os.path.dirname(__file__)
MODEL_PATH = os.path.join(PROJECT_ROOT, "velocity_model.pth")

def create_initial_state():
    return NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )

def run_single_simulation(
    trajectory: dict,
    outage_start: float,
    outage_end: float,
    use_ml: bool = False,
    update_interval: int = 20,
    bias_correction: float = -0.384,
    variance_scale: float = 2.068,
    adaptive_policy: bool = False,
    speed_min_valid: float = 2.0,
    speed_max_valid: float = 35.0,
    max_uncertainty_thresh: float = 10.0,
    gate_threshold: float = 3.0
):
    time = trajectory["time"]
    accel_v = trajectory["accel_v"]
    gyro_v = trajectory["gyro_v"]
    vel_v_ground = trajectory["vel_v"]

    N = len(time)
    dt = time[1] - time[0] if N > 1 else 0.01

    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    ml_estimator = None
    if use_ml:
        ml_estimator = VelocityEstimatorAPI(
            model_path=MODEL_PATH,
            window_size=100,
            update_interval=update_interval
        )
        ml_estimator.samples_since_last_update = update_interval

    vel_est_north = np.zeros(N)
    ml_updates_attempted = 0
    ml_updates_accepted = 0
    ml_predictions = []
    ml_uncertainties = []
    ml_residuals = []
    ml_innovations = []
    ml_rejection_reasons = []

    for i in range(N):
        ts_ns = int(time[i] * 1e9)
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(accel_v[i]),
            gyro_rad_s=tuple(gyro_v[i])
        )

        ins.propagate(imu, q_sensor_to_vehicle=(1.0, 0.0, 0.0, 0.0), propagate_covariance=True)

        if time[i] < outage_start or time[i] > outage_end:
            gnss_vel_ned = (vel_v_ground[i][0], 0.0, 0.0)
            gnss_pos_ned = (0.0, 0.0, 0.0)
            eskf.update_velocity(gnss_vel_ned, np.eye(3) * 0.1)
            eskf.update_position(gnss_pos_ned, np.eye(3) * 0.1)
        else:
            if use_ml and ml_estimator is not None:
                ml_estimator.add_vehicle_frame_sample(
                    tuple(accel_v[i]),
                    tuple(gyro_v[i])
                )

                if ml_estimator.should_update():
                    ml_updates_attempted += 1
                    v_ml_raw, v_var_raw = ml_estimator.estimate_velocity()
                    v_ml_corrected = v_ml_raw - bias_correction
                    v_var_scaled = v_var_raw * variance_scale
                    v_gt = vel_v_ground[i][0]

                    ml_predictions.append(v_ml_corrected)
                    ml_uncertainties.append(np.sqrt(v_var_scaled))
                    ml_residuals.append(v_ml_corrected - v_gt)

                    # Adaptive fallback checks
                    should_apply = True
                    rejection_reason = "accepted"

                    if adaptive_policy:
                        # Check 1: Speed range sanity
                        if v_ml_corrected < speed_min_valid or v_ml_corrected > speed_max_valid:
                            should_apply = False
                            rejection_reason = f"speed_out_of_envelope ({v_ml_corrected:.1f} m/s)"
                        # Check 2: Uncertainty threshold
                        elif np.sqrt(v_var_scaled) > max_uncertainty_thresh:
                            should_apply = False
                            rejection_reason = f"high_uncertainty ({np.sqrt(v_var_scaled):.1f})"
                        # Check 3: Innovation check against current INS state before update
                        current_v_est = ins.state.velocity_mps[0]
                        innov_pre = abs(v_ml_corrected - current_v_est)
                        if innov_pre > 10.0:  # Excessive innovation fallback
                            should_apply = False
                            rejection_reason = f"excessive_innovation ({innov_pre:.1f} m/s)"

                    if should_apply:
                        res = eskf.update_forward_velocity(
                            forward_speed_mps=v_ml_corrected,
                            variance=v_var_scaled,
                            gate=gate_threshold,
                            couple_attitude=False
                        )
                        if res.accepted:
                            ml_updates_accepted += 1
                            ml_innovations.append(float(res.innovation[0]))
                        else:
                            rejection_reason = f"mahalanobis_gate ({res.mahalanobis_dist:.2f} > {gate_threshold})"

                    ml_rejection_reasons.append(rejection_reason)

        vel_est_north[i] = ins.state.velocity_mps[0]

    pos_gt_north = np.zeros(N)
    pos_est_north = np.zeros(N)
    for i in range(1, N):
        dt_i = time[i] - time[i-1]
        pos_gt_north[i] = pos_gt_north[i-1] + vel_v_ground[i-1][0] * dt_i
        pos_est_north[i] = pos_est_north[i-1] + vel_est_north[i-1] * dt_i

    # Evaluate metrics during and end of outage
    outage_mask = (time >= outage_start) & (time <= outage_end)
    final_drift = float(abs(pos_est_north[-1] - pos_gt_north[-1]))
    outage_end_idx = np.where(time <= outage_end)[0][-1]
    drift_at_outage_end = float(abs(pos_est_north[outage_end_idx] - pos_gt_north[outage_end_idx]))

    vel_error = vel_est_north - vel_v_ground[:, 0]
    velocity_rmse = float(np.sqrt(np.mean(vel_error**2)))
    outage_vel_rmse = float(np.sqrt(np.mean(vel_error[outage_mask]**2)))
    max_vel_error = float(np.max(np.abs(vel_error)))

    ml_stats = {}
    if ml_predictions:
        preds = np.array(ml_predictions)
        resids = np.array(ml_residuals)
        uncs = np.array(ml_uncertainties)
        ml_stats = {
            "mean_prediction": float(np.mean(preds)),
            "mean_residual": float(np.mean(resids)),
            "residual_rmse": float(np.sqrt(np.mean(resids**2))),
            "max_abs_residual": float(np.max(np.abs(resids))),
            "mean_uncertainty_std": float(np.mean(uncs)),
            "attempted_updates": ml_updates_attempted,
            "accepted_updates": ml_updates_accepted,
            "acceptance_rate": float(ml_updates_accepted / max(1, ml_updates_attempted))
        }
        if len(resids) > 1:
            var_res = np.var(resids)
            if var_res > 1e-12:
                ml_stats["residual_autocorr_lag1"] = float(np.corrcoef(resids[:-1], resids[1:])[0, 1])
            else:
                ml_stats["residual_autocorr_lag1"] = 1.0

    return {
        "final_drift_m": final_drift,
        "drift_at_outage_end_m": drift_at_outage_end,
        "velocity_rmse_m_s": velocity_rmse,
        "outage_velocity_rmse_m_s": outage_vel_rmse,
        "max_velocity_error_m_s": max_vel_error,
        "ml_updates_accepted": ml_updates_accepted,
        "ml_updates_attempted": ml_updates_attempted,
        "ml_stats": ml_stats,
        "distance_travelled_m": float(pos_gt_north[-1])
    }

def run_full_suite():
    outage_durations = [5, 15, 30]
    max_speeds = [10.0, 20.0, 30.0]
    total_duration = 50.0
    outage_start = 10.0

    print("=" * 80)
    print("PHASE 11: FULL 9-SCENARIO EVALUATION WITH TRAINED ML MODEL")
    print("=" * 80)

    # 1. Base test: Policy A (interval=20, standard)
    results_policy_20 = []
    results_policy_30 = []
    results_policy_40 = []
    results_adaptive = []

    for dur in outage_durations:
        for spd in max_speeds:
            gen = SyntheticTrajectoryGenerator(dt=0.01, seed=42)
            traj = gen.generate_straight_accel_decel(duration=total_duration, max_speed=spd)
            outage_end = outage_start + dur

            # Baseline ESKF-only
            base = run_single_simulation(traj, outage_start, outage_end, use_ml=False)

            # Policy A: update_interval=20
            p20 = run_single_simulation(traj, outage_start, outage_end, use_ml=True, update_interval=20)

            # Policy B: update_interval=30
            p30 = run_single_simulation(traj, outage_start, outage_end, use_ml=True, update_interval=30)

            # Policy C: update_interval=40
            p40 = run_single_simulation(traj, outage_start, outage_end, use_ml=True, update_interval=40)

            # Policy D: Adaptive / Confidence-gated
            p_adapt = run_single_simulation(traj, outage_start, outage_end, use_ml=True, update_interval=20, adaptive_policy=True)

            scenario_key = f"{dur}s_outage_{int(spd)}mps"

            rec_20 = {
                "scenario": scenario_key,
                "outage_duration_s": dur,
                "max_speed_mps": spd,
                "distance_m": base["distance_travelled_m"],
                "eskf_only": base,
                "eskf_ml": p20,
                "drift_change": p20["final_drift_m"] - base["final_drift_m"],
                "vel_rmse_change": p20["velocity_rmse_m_s"] - base["velocity_rmse_m_s"],
                "non_degrading": bool((p20["final_drift_m"] <= base["final_drift_m"]) and (p20["velocity_rmse_m_s"] <= base["velocity_rmse_m_s"]))
            }
            results_policy_20.append(rec_20)

            rec_30 = {
                "scenario": scenario_key,
                "outage_duration_s": dur,
                "max_speed_mps": spd,
                "drift_change": p30["final_drift_m"] - base["final_drift_m"],
                "vel_rmse_change": p30["velocity_rmse_m_s"] - base["velocity_rmse_m_s"],
                "non_degrading": bool((p30["final_drift_m"] <= base["final_drift_m"]) and (p30["velocity_rmse_m_s"] <= base["velocity_rmse_m_s"])),
                "ml_updates": p30["ml_updates_accepted"],
                "final_drift": p30["final_drift_m"],
                "vel_rmse": p30["velocity_rmse_m_s"]
            }
            results_policy_30.append(rec_30)

            rec_40 = {
                "scenario": scenario_key,
                "outage_duration_s": dur,
                "max_speed_mps": spd,
                "drift_change": p40["final_drift_m"] - base["final_drift_m"],
                "vel_rmse_change": p40["velocity_rmse_m_s"] - base["velocity_rmse_m_s"],
                "non_degrading": bool((p40["final_drift_m"] <= base["final_drift_m"]) and (p40["velocity_rmse_m_s"] <= base["velocity_rmse_m_s"])),
                "ml_updates": p40["ml_updates_accepted"],
                "final_drift": p40["final_drift_m"],
                "vel_rmse": p40["velocity_rmse_m_s"]
            }
            results_policy_40.append(rec_40)

            rec_adapt = {
                "scenario": scenario_key,
                "outage_duration_s": dur,
                "max_speed_mps": spd,
                "drift_change": p_adapt["final_drift_m"] - base["final_drift_m"],
                "vel_rmse_change": p_adapt["velocity_rmse_m_s"] - base["velocity_rmse_m_s"],
                "non_degrading": bool((p_adapt["final_drift_m"] <= base["final_drift_m"]) and (p_adapt["velocity_rmse_m_s"] <= base["velocity_rmse_m_s"])),
                "ml_updates": p_adapt["ml_updates_accepted"],
                "final_drift": p_adapt["final_drift_m"],
                "vel_rmse": p_adapt["velocity_rmse_m_s"]
            }
            results_adaptive.append(rec_adapt)

    print("\n--- SUMMARY OF POLICY A (update_interval=20) ---")
    print(f"{'Scenario':<20} | {'ESKF Drift':<12} | {'ML Drift':<12} | {'Drift Diff':<12} | {'ESKF Vel':<10} | {'ML Vel':<10} | {'Updates':<8} | {'Status'}")
    print("-" * 105)
    for r in results_policy_20:
        base = r["eskf_only"]
        ml = r["eskf_ml"]
        status = "[PASS]" if r["non_degrading"] else "[FAIL]"
        print(f"{r['scenario']:<20} | {base['final_drift_m']:<12.3f} | {ml['final_drift_m']:<12.3f} | {r['drift_change']:<+12.3f} | {base['velocity_rmse_m_s']:<10.3f} | {ml['velocity_rmse_m_s']:<10.3f} | {ml['ml_updates_accepted']:<8} | {status}")

    print("\n--- COMPARISON OF POLICIES (20 vs 30 vs 40 vs Adaptive) ---")
    print(f"{'Scenario':<20} | {'Base Drift':<10} | {'Int=20':<10} | {'Int=30':<10} | {'Int=40':<10} | {'Adaptive':<10}")
    print("-" * 80)
    for i in range(len(results_policy_20)):
        scen = results_policy_20[i]["scenario"]
        b_d = results_policy_20[i]["eskf_only"]["final_drift_m"]
        d20 = results_policy_20[i]["eskf_ml"]["final_drift_m"]
        d30 = results_policy_30[i]["final_drift"]
        d40 = results_policy_40[i]["final_drift"]
        dadapt = results_adaptive[i]["final_drift"]
        print(f"{scen:<20} | {b_d:<10.3f} | {d20:<10.3f} | {d30:<10.3f} | {d40:<10.3f} | {dadapt:<10.3f}")

    with open("phase11_trained_model_results.json", "w") as f:
        json.dump({
            "policy_20": results_policy_20,
            "policy_30": results_policy_30,
            "policy_40": results_policy_40,
            "adaptive": results_adaptive
        }, f, indent=2)

    print("\nResults written to phase11_trained_model_results.json")

if __name__ == "__main__":
    run_full_suite()
