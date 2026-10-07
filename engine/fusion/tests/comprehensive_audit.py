"""
Comprehensive audit of all 16 benchmark sessions.
Identifies root causes of drift discrepancies, speed_scale convergence issues,
and fusion engine stability problems.
"""
import sys
import os

# Ensure correct path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.dataset_splits import TEST_SESSIONS
import numpy as np

def comprehensive_audit():
    """Run comprehensive audit on all 16 sessions."""

    print("=" * 80)
    print("COMPREHENSIVE AUDIT: All 16 Benchmark Sessions")
    print("=" * 80)

    # Build session list
    sessions = []

    # Cars from test set
    for driver, session in TEST_SESSIONS:
        sessions.append({"category": "car", "driver": driver, "session": session})

    # Two-wheelers
    sessions.extend([
        {"category": "two_wheeler", "session": "session1"},
        {"category": "two_wheeler", "session": "session2"}
    ])

    print(f"\nAuditing {len(sessions)} sessions...")
    print("=" * 80)

    results = []
    issues_found = []

    for i, cfg in enumerate(sessions, 1):
        session_name = cfg.get("session", "UNKNOWN")
        category = cfg.get("category", "UNKNOWN")

        print(f"\n[{i}/{len(sessions)}] {session_name} ({category})")
        print("-" * 80)

        try:
            res = evaluate_dead_reckoning_session(cfg)

            drift_pct = float(res["drift_pct"])
            final_error_m = float(res["final_error_m"])
            outage_dist_m = float(res["outage_dist_m"])
            nis_pass_rate = float(res["nis_pass_rate"])
            official_pass = bool(res["official_pass"])

            print(f"  Drift: {drift_pct:.2f}% | Error: {final_error_m:.2f}m | Dist: {outage_dist_m:.2f}m")
            print(f"  NIS Pass Rate: {nis_pass_rate:.1f}% | Official: {'PASS' if official_pass else 'FAIL'}")

            # Calculate error-to-distance ratio
            err_ratio = final_error_m / outage_dist_m if outage_dist_m > 0 else 0.0

            # Detect discrepancies
            discrepancy_flags = []

            # Flag 1: Low drift % but high final error (ratio mismatch)
            if drift_pct < 10.0 and final_error_m > 100.0:
                discrepancy_flags.append(f"LOW_DRIFT_HIGH_ERROR (drift={drift_pct:.1f}%, error={final_error_m:.1f}m)")

            # Flag 2: High drift % (exceeds target)
            if drift_pct > 10.0:
                discrepancy_flags.append(f"HIGH_DRIFT (drift={drift_pct:.1f}% > 10%)")

            # Flag 3: Very low NIS pass rate (gating issue)
            if nis_pass_rate < 50.0:
                discrepancy_flags.append(f"LOW_NIS_PASS_RATE ({nis_pass_rate:.1f}%)")

            # Flag 4: Very high NIS pass rate (possible no gating)
            if nis_pass_rate > 99.0:
                discrepancy_flags.append(f"EXCESSIVE_NIS_PASS_RATE ({nis_pass_rate:.1f}%)")

            if discrepancy_flags:
                issues_found.append({
                    "session": session_name,
                    "category": category,
                    "drift_pct": drift_pct,
                    "final_error_m": final_error_m,
                    "outage_dist_m": outage_dist_m,
                    "err_ratio": err_ratio,
                    "nis_pass_rate": nis_pass_rate,
                    "flags": discrepancy_flags
                })
                print(f"  [!] ISSUES DETECTED:")
                for flag in discrepancy_flags:
                    print(f"    - {flag}")
            else:
                print(f"  ✓ No major issues")

            results.append({
                "session": session_name,
                "category": category,
                "drift_pct": drift_pct,
                "final_error_m": final_error_m,
                "outage_dist_m": outage_dist_m,
                "err_ratio": err_ratio,
                "nis_pass_rate": nis_pass_rate,
                "official_pass": official_pass,
                "status": "OK"
            })

        except Exception as e:
            print(f"  ✗ ERROR: {str(e)[:100]}")
            results.append({
                "session": session_name,
                "category": category,
                "status": f"ERROR: {str(e)[:100]}"
            })

    # Summary statistics
    print("\n" + "=" * 80)
    print("SUMMARY STATISTICS")
    print("=" * 80)

    valid_results = [r for r in results if r["status"] == "OK"]
    car_results = [r for r in valid_results if r["category"] == "car"]
    tw_results = [r for r in valid_results if r["category"] == "two_wheeler"]

    if valid_results:
        drifts = [r["drift_pct"] for r in valid_results]
        errors = [r["final_error_m"] for r in valid_results]
        nis_rates = [r["nis_pass_rate"] for r in valid_results]

        print(f"\nAll Sessions ({len(valid_results)}):")
        print(f"  Drift %: mean={np.mean(drifts):.2f}, median={np.median(drifts):.2f}, range=[{np.min(drifts):.2f}, {np.max(drifts):.2f}]")
        print(f"  Error (m): mean={np.mean(errors):.2f}, median={np.median(errors):.2f}, range=[{np.min(errors):.2f}, {np.max(errors):.2f}]")
        print(f"  NIS Rate: mean={np.mean(nis_rates):.1f}%, range=[{np.min(nis_rates):.1f}, {np.max(nis_rates):.1f}]")
        print(f"  Official Pass: {sum(1 for r in valid_results if r['official_pass'])}/{len(valid_results)}")

    if car_results:
        car_drifts = [r["drift_pct"] for r in car_results]
        print(f"\nCars ({len(car_results)}):")
        print(f"  Drift %: mean={np.mean(car_drifts):.2f}, median={np.median(car_drifts):.2f}")
        print(f"  Pass: {sum(1 for r in car_results if r['official_pass'])}/{len(car_results)}")

    if tw_results:
        tw_drifts = [r["drift_pct"] for r in tw_results]
        print(f"\nTwo-Wheelers ({len(tw_results)}):")
        print(f"  Drift %: mean={np.mean(tw_drifts):.2f}, median={np.median(tw_drifts):.2f}")
        print(f"  Pass: {sum(1 for r in tw_results if r['official_pass'])}/{len(tw_results)}")

    # Root cause analysis
    print("\n" + "=" * 80)
    print("ROOT CAUSE ANALYSIS")
    print("=" * 80)

    if issues_found:
        print(f"\nFound {len(issues_found)} sessions with issues:\n")

        for issue in issues_found:
            print(f"{issue['session']} ({issue['category']}):")
            print(f"  Drift: {issue['drift_pct']:.2f}% | Error: {issue['final_error_m']:.2f}m | Dist: {issue['outage_dist_m']:.2f}m")
            print(f"  Error/Distance Ratio: {issue['err_ratio']:.3f}")
            print(f"  NIS Pass Rate: {issue['nis_pass_rate']:.1f}%")
            print(f"  Issues:")
            for flag in issue['flags']:
                print(f"    - {flag}")

            # Infer root cause
            root_causes = []

            if any("HIGH_DRIFT" in f for f in issue['flags']):
                root_causes.append("speed_scale learning issue (pre-outage speed too low or AI model mismatch)")

            if any("LOW_DRIFT_HIGH_ERROR" in f for f in issue['flags']):
                root_causes.append("drift calculation issue (stationary segment or distance underestimated)")

            if any("LOW_NIS_PASS_RATE" in f for f in issue['flags']):
                root_causes.append("EKF divergence or excessive gating rejection")

            if any("EXCESSIVE_NIS_PASS_RATE" in f for f in issue['flags']):
                root_causes.append("gating threshold too loose or covariance underestimated")

            if root_causes:
                print(f"  Likely Root Cause(s):")
                for rc in root_causes:
                    print(f"    → {rc}")
            print()
    else:
        print("\n✓ No critical issues found in any session.")

    # Recommendations
    print("=" * 80)
    print("RECOMMENDATIONS")
    print("=" * 80)

    # Analyze patterns
    high_drift_sessions = [r for r in valid_results if r["drift_pct"] > 10.0]
    low_nis_sessions = [r for r in valid_results if r["nis_pass_rate"] < 70.0]

    if high_drift_sessions:
        print(f"\n1. HIGH DRIFT ({len(high_drift_sessions)} sessions):")
        print(f"   → Lower speed gate for two-wheelers (0.5 m/s instead of 1.0 m/s)")
        print(f"   → Use median AI speed as fallback for speed_scale (not GNSS ratio)")
        print(f"   → Increase pre-outage learning window from 30s to 60s")

    if low_nis_sessions:
        print(f"\n2. LOW NIS PASS RATE ({len(low_nis_sessions)} sessions):")
        print(f"   → Review EKF Q matrix tuning (sigma_acc, sigma_gyro)")
        print(f"   → Check for covariance overflow (EKF instability)")

    print("\n3. GENERAL TUNING:")
    print(f"   → Verify AI speed filter training on two-wheeler data")
    print(f"   → Add speed_scale convergence monitoring")
    print(f"   → Enable speed_scale history logging in fusion engine")

    print("\n" + "=" * 80)

    return {
        "total_sessions": len(sessions),
        "valid_sessions": len(valid_results),
        "issues_found": len(issues_found),
        "high_drift_count": len(high_drift_sessions) if valid_results else 0,
        "low_nis_count": len(low_nis_sessions) if valid_results else 0,
        "issue_sessions": [i["session"] for i in issues_found]
    }

if __name__ == "__main__":
    summary = comprehensive_audit()
    print("\n=== AUDIT COMPLETE ===")
    print(f"Sessions audited: {summary['valid_sessions']}/{summary['total_sessions']}")
    print(f"Issues found: {summary['issues_found']}")
    if summary['issue_sessions']:
        print(f"Sessions with issues: {', '.join(summary['issue_sessions'])}")
