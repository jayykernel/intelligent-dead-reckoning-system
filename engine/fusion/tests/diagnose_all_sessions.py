"""
engine/fusion/tests/diagnose_all_sessions.py

Comprehensive diagnostic analysis across all test sessions.
Computes error progression profile over the 60s outage window.
"""

import os
import sys
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from eval.run_full_benchmark import (
    evaluate_dead_reckoning_session,
    evaluate_edge_fog_session,
)
from training.dataset_splits import TEST_SESSIONS

CAR_TEST_SESSIONS = [{"category": "car", "driver": d, "session": s} for d, s in TEST_SESSIONS]
TW_TEST_SESSIONS = [{"category": "two_wheeler", "session": "session1"}, {"category": "two_wheeler", "session": "session2"}]

def run_diagnostics():
    print("=" * 80)
    print("COMPREHENSIVE MULTI-SESSION DIAGNOSTIC ANALYSIS")
    print("=" * 80)

    # 1. Car Sessions
    car_results = {}
    for session_config in CAR_TEST_SESSIONS:
        print(f"\n>>> Diagnosing Car Session: {session_config['session']} ...")
        res = evaluate_dead_reckoning_session(session_config)
        car_results[session_config['session']] = res

    # 2. Two-Wheeler Sessions
    tw_results = {}
    for session_config in TW_TEST_SESSIONS:
        print(f"\n>>> Diagnosing Two-Wheeler Session: {session_config['session']} ...")
        res = evaluate_dead_reckoning_session(session_config)
        tw_results[session_config['session']] = res

    # 3. Edge Engine
    print("\n>>> Diagnosing Edge Engine: S1 ...")
    edge_res = evaluate_edge_fog_session()

    print("\n" + "=" * 80)
    print("DIAGNOSTIC SUMMARY TABLE")
    print("=" * 80)
    print(f"{'Session':<12} | {'Category':<12} | {'Dist (m)':<9} | {'Final Err':<10} | {'Drift %':<8} | {'Status':<6}")
    print("-" * 80)

    all_drifts = []
    for s_name, res in car_results.items():
        if res:
            status = "PASS" if res["drift_pct"] <= 10.0 else "FAIL"
            print(f"{s_name:<12} | {'Car':<12} | {res['outage_dist_m']:<9.2f} | {res['final_error_m']:<10.2f} | {res['drift_pct']:<7.2f}% | {status:<6}")
            all_drifts.append((s_name, res["drift_pct"]))

    for s_name, res in tw_results.items():
        if res:
            status = "PASS" if res["drift_pct"] <= 10.0 else "FAIL"
            print(f"{s_name:<12} | {'TwoWheeler':<12} | {res['outage_dist_m']:<9.2f} | {res['final_error_m']:<10.2f} | {res['drift_pct']:<7.2f}% | {status:<6}")
            all_drifts.append((s_name, res["drift_pct"]))

    if edge_res:
        status = "PASS" if edge_res["drift_pct"] <= 10.0 else "FAIL"
        print(f"{'S1 (FOG)':<12} | {'Edge FOG':<12} | {edge_res['outage_dist_m']:<9.2f} | {edge_res['final_error_m']:<10.2f} | {edge_res['drift_pct']:<7.2f}% | {status:<6}")
        all_drifts.append(("S1 (FOG)", edge_res["drift_pct"]))

    print("-" * 80)
    max_drift = max(d[1] for d in all_drifts)
    worst_session = [d[0] for d in all_drifts if d[1] == max_drift][0]
    print(f"Max Drift Across All 16 Sessions: {max_drift:.2f}% (Worst: {worst_session})")
    print(f"Sessions Meeting Target (<= 10.0%): {sum(1 for d in all_drifts if d[1] <= 10.0)} / {len(all_drifts)}")
    print("=" * 80)

if __name__ == "__main__":
    run_diagnostics()
