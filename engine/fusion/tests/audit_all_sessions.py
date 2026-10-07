"""
Deep-dive audit of all 16 benchmark sessions.
Identifies root causes of drift discrepancies, speed_scale convergence,
AI speed consistency, and fusion engine stability.
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.dataset_splits import TEST_SESSIONS
import json
import numpy as np

def audit_session(cfg):
    """Run a single session and extract audit metrics."""
    try:
        res = evaluate_dead_reckoning_session(cfg)
        return {
            "session": res["session"],
            "category": res["category"],
            "outage_dist_m": float(res["outage_dist_m"]),
            "final_error_m": float(res["final_error_m"]),
            "drift_pct": float(res["drift_pct"]),
            "is_stationary": bool(res["is_stationary"]),
            "official_pass": bool(res["official_pass"]),
            "stretch_pass": bool(res["stretch_pass"]),
            "gnss_total": int(res["gnss_total"]),
            "gnss_passed": int(res["gnss_passed"]),
            "nis_pass_rate": float(res["nis_pass_rate"]),
            "error_to_distance_ratio": float(res["final_error_m"] / res["outage_dist_m"]) if res["outage_dist_m"] > 0 else 0.0,
            "status": "OK"
        }
    except Exception as e:
        return {
            "session": cfg.get("session", "UNKNOWN"),
            "category": cfg.get("category", "UNKNOWN"),
            "status": f"ERROR: {str(e)[:100]}"
        }

def main():
    print("=" * 80)
    print("AUDIT: All 16 Benchmark Sessions")
    print("=" * 80)

    # Build session list
    sessions_to_audit = []

    # Cars from test set
    for driver, session in TEST_SESSIONS:
        sessions_to_audit.append({"category": "car", "driver": driver, "session": session})

    # Two-wheelers
    sessions_to_audit.extend([
        {"category": "two_wheeler", "session": "session1"},
        {"category": "two_wheeler", "session": "session2"}
    ])

    print(f"\nAuditing {len(sessions_to_audit)} sessions...\n")

    results = []
    for i, cfg in enumerate(sessions_to_audit, 1):
        session_name = cfg.get("session", "UNKNOWN")
        print(f"[{i}/{len(sessions_to_audit)}] {session_name}...", end=" ", flush=True)
        res = audit_session(cfg)
        results.append(res)
        print(f"✓ {res.get('drift_pct', 'N/A'):.2f}%" if "drift_pct" in res else "✗")

    # Analyze results
    print("\n" + "=" * 80)
    print("AUDIT RESULTS")
    print("=" * 80)

    car_results = [r for r in results if r.get("category") == "car" and "drift_pct" in r]
    tw_results = [r for r in results if r.get("category") == "two_wheeler" and "drift_pct" in r]

    print(f"\nCARS ({len(car_results)} sessions):")
    print("-" * 80)

    # Sort by drift
    car_results_sorted = sorted(car_results, key=lambda x: x["drift_pct"])
    for r in car_results_sorted:
        drift_status = "✓ PASS" if r["official_pass"] else "✗ FAIL"
        nis_status = "✓ Good" if r["nis_pass_rate"] > 90 else "⚠ Moderate" if r["nis_pass_rate"] > 70 else "✗ Poor"
        print(f"  {r['session']:12s} | Drift: {r['drift_pct']:7.2f}% {drift_status} | "
              f"Error: {r['final_error_m']:6.2f}m | Dist: {r['outage_dist_m']:7.2f}m | "
              f"NIS: {r['nis_pass_rate']:5.1f}% {nis_status}")

    print(f"\nTWO-WHEELERS ({len(tw_results)} sessions):")
    print("-" * 80)

    tw_results_sorted = sorted(tw_results, key=lambda x: x["drift_pct"])
    for r in tw_results_sorted:
        drift_status = "✓ PASS" if r["official_pass"] else "✗ FAIL"
        nis_status = "✓ Good" if r["nis_pass_rate"] > 90 else "⚠ Moderate" if r["nis_pass_rate"] > 70 else "✗ Poor"
        print(f"  {r['session']:12s} | Drift: {r['drift_pct']:7.2f}% {drift_status} | "
              f"Error: {r['final_error_m']:6.2f}m | Dist: {r['outage_dist_m']:7.2f}m | "
              f"NIS: {r['nis_pass_rate']:5.1f}% {nis_status}")

    # Statistics
    print("\n" + "=" * 80)
    print("STATISTICS")
    print("=" * 80)

    all_valid = [r for r in results if "drift_pct" in r]
    if all_valid:
        drifts = [r["drift_pct"] for r in all_valid]
        nis_rates = [r["nis_pass_rate"] for r in all_valid]

        print(f"\nDrift % (all {len(all_valid)} sessions):")
        print(f"  Mean: {np.mean(drifts):.2f}%")
        print(f"  Median: {np.median(drifts):.2f}%")
        print(f"  Std Dev: {np.std(drifts):.2f}%")
        print(f"  Min: {np.min(drifts):.2f}%")
        print(f"  Max: {np.max(drifts):.2f}%")
        print(f"  Official Pass (<=10%): {sum(1 for r in all_valid if r['official_pass'])}/{len(all_valid)}")
        print(f"  Stretch Pass (<=2%): {sum(1 for r in all_valid if r['stretch_pass'])}/{len(all_valid)}")

        print(f"\nNIS Acceptance Rate (%):")
        print(f"  Mean: {np.mean(nis_rates):.1f}%")
        print(f"  Median: {np.median(nis_rates):.1f}%")
        print(f"  Min: {np.min(nis_rates):.1f}%")
        print(f"  Max: {np.max(nis_rates):.1f}%")
        print(f"  All > 50%: {'✓ Yes' if all(r > 50 for r in nis_rates) else '✗ No'}")

    # Export JSON
    with open("engine/fusion/tests/audit_all_sessions_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to engine/fusion/tests/audit_all_sessions_results.json")

    # Identify outliers
    print("\n" + "=" * 80)
    print("OUTLIER DETECTION")
    print("=" * 80)

    if car_results:
        car_drifts = [r["drift_pct"] for r in car_results]
        q1, q3 = np.percentile(car_drifts, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        car_outliers = [r for r in car_results if r["drift_pct"] > upper_bound]
        if car_outliers:
            print(f"\nCar outliers (drift > {upper_bound:.2f}%):")
            for r in car_outliers:
                print(f"  {r['session']}: {r['drift_pct']:.2f}% (Error: {r['final_error_m']:.2f}m, Dist: {r['outage_dist_m']:.2f}m)")

    if tw_results:
        tw_drifts = [r["drift_pct"] for r in tw_results]
        if len(tw_drifts) > 1:
            q1, q3 = np.percentile(tw_drifts, [25, 75])
            iqr = q3 - q1
            if iqr > 0:
                lower_bound = q1 - 1.5 * iqr
                upper_bound = q3 + 1.5 * iqr

                tw_outliers = [r for r in tw_results if r["drift_pct"] > upper_bound]
                if tw_outliers:
                    print(f"\nTwo-wheeler outliers (drift > {upper_bound:.2f}%):")
                    for r in tw_outliers:
                        print(f"  {r['session']}: {r['drift_pct']:.2f}% (Error: {r['final_error_m']:.2f}m, Dist: {r['outage_dist_m']:.2f}m)")

    print("\n" + "=" * 80)

if __name__ == "__main__":
    main()
