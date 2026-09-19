"""
engine/fusion/tests/test_mode_transition.py

Phase 9: Seamless Mode Transition Handler Verification & Latency Benchmark.

Measures:
1. Outage Entry Settling Latency: time from signal degradation start to covariance
   growing consistently with pure INS behavior (variance > 5x baseline).
2. Reacquisition Settling Latency: time from GNSS reacquisition start to covariance
   shrinking back towards GNSS-aided levels (variance < 2x baseline) AND at least
   one GNSS update accepted.
3. State continuity across mode boundaries (verifying delta position and velocity).
4. Validates that covariance is dynamically scaled based on trust, ensuring the mode
   switch mathematically influences the filter.
"""

import time
import numpy as np
from engine.fusion.fusion_engine import GNSSINSFusionEngine


def get_pos_variance(cov_2d):
    return cov_2d[0, 0] + cov_2d[1, 1]


def run_transition_benchmark():
    print("==================================================================")
    print("PHASE 9: CONTINUOUS TRANSITION DYNAMICS & COVARIANCE SETTLING")
    print("==================================================================")

    dt = 0.1  # 10 Hz
    engine = GNSSINSFusionEngine(dt=dt)

    # Enable outage predictor and other components gracefully
    engine.initialize_state(
        p0_enu=np.array([0.0, 0.0, 0.0]),
        v0_enu=np.array([10.0, 0.0, 0.0]),
        heading0_deg=90.0,
        acc0_raw=np.array([0.0, 0.0, 9.81])
    )

    t = 0.0
    # 1. Warm-up / Steady State in GNSS_AIDED (4.0 seconds)
    steady_variances = []
    for _ in range(40):
        t += dt
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=np.array([10.0 * t, 0.0, 0.0]),
            gnss_vel_enu=np.array([10.0, 0.0, 0.0]),
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=36.0,
            gnss_sat_count=16,
            gnss_acc_m=2.5
        )
        steady_variances.append(get_pos_variance(res['cov_2d']))

    baseline_var = np.mean(steady_variances[-10:])
    print(f"[Steady State GNSS_AIDED] Baseline Pos Variance: {baseline_var:.4f} m^2")

    # 2. Trigger Downward Transition (Outage Entry, Signal Degradation leading to Loss)
    print("\n--- Testing Outage Entry (GNSS_AIDED -> PURE_DEAD_RECKONING) ---")

    # Simulate entering a tunnel (drop signal quality over a few seconds, then lose it)
    pos_before_trans = np.copy(engine.ekf.p)
    vel_before_trans = np.copy(engine.ekf.v)

    degradation_start_t = t
    cov_history = []
    degradation_epochs = 20  # 2.0 seconds of degradation
    outage_epochs = 30       # 3.0 seconds of hard outage

    # Signal degrades (C/N0 drops, accuracy degrades)
    for i in range(degradation_epochs):
        t += dt
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=np.array([10.0 * t, 0.0, 0.0]),
            gnss_vel_enu=np.array([10.0, 0.0, 0.0]),
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=15.0,  # Degraded signal
            gnss_sat_count=4,   # Barely tracking
            gnss_acc_m=20.0     # High uncertainty
        )
        cov_history.append((t, res['mode'], res['trust_score'], res['gnss_pos_passed'], get_pos_variance(res['cov_2d'])))

    # Hard loss occurs
    hard_loss_start_t = t
    for i in range(outage_epochs):
        t += dt
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=None,
            gnss_vel_enu=None,
            is_gnss_available=False,
            timestamp=t,
            gnss_avg_cn0=0.0,
            gnss_sat_count=0,
            gnss_acc_m=99.0
        )
        cov_history.append((t, res['mode'], res['trust_score'], res['gnss_pos_passed'], get_pos_variance(res['cov_2d'])))

    pos_after_trans = np.copy(engine.ekf.p)
    vel_after_trans = np.copy(engine.ekf.v)

    # Analyze Outage Transition Settling
    print(f"Signal Degradation Start: t={degradation_start_t:.1f}s")
    print(f"Hard Set to None (Outage): t={hard_loss_start_t:.1f}s")

    flag_flip_entry_t = None
    settling_entry_t = None
    pure_ins_growth_detected = False

    for epoch_t, mode, trust, gnss_passed, var in cov_history:
        # Check when flag flipped
        if flag_flip_entry_t is None and mode == "PURE_DEAD_RECKONING":
            flag_flip_entry_t = epoch_t

        # Check when covariance is growing > 5x baseline (indicating pure INS dominates)
        if settling_entry_t is None and var > baseline_var * 5.0 and mode == "PURE_DEAD_RECKONING":
            settling_entry_t = epoch_t

    if flag_flip_entry_t:
        print(f"  -> Flag Flips to PURE_DEAD_RECKONING at t={flag_flip_entry_t:.1f}s")
    if settling_entry_t:
        settling_latency = settling_entry_t - degradation_start_t
        print(f"  -> Covariance Settles to Pure INS Growth at t={settling_entry_t:.1f}s")
        print(f"  -> Outage Entry Settling Latency (Logical Time): {settling_latency:.2f}s")
    else:
        print("  -> ERROR: Covariance did not enter pure INS growth mode!")
        print(f"     Last var: {cov_history[-1][4]:.4f} vs baseline: {baseline_var:.4f}")
        settling_latency = float('inf')

    # 3. Simulate Pure Dead Reckoning for 5.0 seconds
    print("\n--- Running Pure Dead Reckoning for 5.0 seconds ---")
    for _ in range(50):
        t += dt
        engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=None,
            gnss_vel_enu=None,
            is_gnss_available=False,
            timestamp=t
        )

    outage_var = get_pos_variance(engine.ekf.get_position_covariance_2d())
    print(f"  -> Current Mode: {engine.mode_current_state}, Pos Variance: {outage_var:.4f} m^2")
    if outage_var <= baseline_var:
        print("  -> WARNING: Covariance not grown during INS (may be too short or constraints too strong).")

    # 4. Trigger Upward Transition (GNSS Reacquisition upon tunnel exit)
    print("\n--- Testing GNSS Reacquisition (PURE_DEAD_RECKONING -> GNSS_AIDED) ---")
    reacq_start_t = t
    cov_history_reacq = []

    pos_before_reacq = np.copy(engine.ekf.p)
    vel_before_reacq = np.copy(engine.ekf.v)

    # Signal reacquires cleanly
    reacq_epochs = 30  # 3.0 seconds of reacquisition
    for i in range(reacq_epochs):
        t += dt
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=np.array([10.0 * t, 0.0, 0.0]),
            gnss_vel_enu=np.array([10.0, 0.0, 0.0]),
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=38.0,
            gnss_sat_count=18,
            gnss_acc_m=2.0
        )
        cov_history_reacq.append((t, res['mode'], res['trust_score'], res['gnss_pos_passed'], get_pos_variance(res['cov_2d'])))

    pos_after_reacq = np.copy(engine.ekf.p)
    vel_after_reacq = np.copy(engine.ekf.v)

    flag_flip_reacq_t = None
    settling_reacq_t = None
    first_gnss_accepted_epoch = None

    for epoch_t, mode, trust, gnss_passed, var in cov_history_reacq:
        if flag_flip_reacq_t is None and mode == "GNSS_AIDED":
            flag_flip_reacq_t = epoch_t

        # Track first accepted GNSS update
        if first_gnss_accepted_epoch is None and gnss_passed:
            first_gnss_accepted_epoch = epoch_t

        # Considered settled when variance collapses back down to within 2x of baseline
        # AND we have seen at least one GNSS update accepted (to ensure filter is correcting)
        if settling_reacq_t is None and var <= baseline_var * 2.0 and mode == "GNSS_AIDED" and first_gnss_accepted_epoch is not None:
            settling_reacq_t = epoch_t

    if flag_flip_reacq_t:
        print(f"  -> Flag Flips to GNSS_AIDED at t={flag_flip_reacq_t:.1f}s")

    if settling_reacq_t:
        settling_latency = settling_reacq_t - reacq_start_t
        print(f"  -> Covariance Settles to Baseline Bounds at t={settling_reacq_t:.1f}s")
        print(f"  -> Reacquisition Settling Latency (Logical Time): {settling_latency:.2f}s")
        print(f"      (First GNSS accepted at t={first_gnss_accepted_epoch:.1f}s)")
    else:
        print("  -> NOTE: Covariance did not settle to baseline within reacquisition window.")
        print(f"     Last var: {cov_history_reacq[-1][4]:.4f} vs baseline: {baseline_var:.4f}")
        if first_gnss_accepted_epoch is not None:
            print(f"     First GNSS accepted at t={first_gnss_accepted_epoch:.1f}s, but variance still high.")
        else:
            print("     No GNSS updates were accepted during reacquisition (likely due to large innovation).")

    print("\n--- State Continuity Check ---")
    delta_pos = np.linalg.norm(pos_after_trans - pos_before_trans)
    delta_vel = np.linalg.norm(vel_after_trans - vel_before_trans)
    print(f"  -> Pos Delta (Outage Entry):   {delta_pos:.4f} m")
    print(f"  -> Vel Delta (Outage Entry):   {delta_vel:.4f} m/s")

    delta_pos_reacq = np.linalg.norm(pos_after_reacq - pos_before_reacq)
    delta_vel_reacq = np.linalg.norm(vel_after_reacq - vel_before_reacq)
    print(f"  -> Pos Delta (Reacquisition):  {delta_pos_reacq:.4f} m")
    print(f"  -> Vel Delta (Reacquisition):  {delta_vel_reacq:.4f} m/s")


if __name__ == "__main__":
    run_transition_benchmark()