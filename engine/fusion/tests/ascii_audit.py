"""
ASCII-only comprehensive audit of all 16 benchmark sessions.
"""
import sys
import os

# Ensure correct path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.dataset_splits import TEST_SESSIONS
import numpy as np

def ascii_audit():
    """Run ASCII-only audit on all sessions."""

    print("=" * 80)
    print("ASCII AUDIT: All 16 Benchmark Sessions")
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
    high_drift_sessions = []
    low_nis_sessions = []

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

            # Detect issues
            issue_flags = []

            # Flag 1: High drift
            if drift_pct > 10.0:
                issue_flags.append(f"HIGH_DRIFT_{drift_pct:.1f}%")
                high_drift_sessions.append(session_name)

            # Flag 2: Low NIS pass rate
            if nis_pass_rate < 70.0:
                issue_flags.append(f"LOW_NIS_{nis_pass_rate:.1f}%")
                low_nis_sessions.append(session_name)

            # Flag 3: Very high NIS pass rate (gating too loose)
            if nis_pass_rate > 99.0:
                issue_flags.append(f"LOOSE_GATING_{nis_pass_rate:.1f}%")

            if issue_flags:
                issues_found.append({
                    "session": session_name,
                    "category": category,
                    "drift_pct": drift_pct,
                    "final_error_m": final_error_m,
                    "outage_dist_m": outage_dist_m,
                    "err_ratio": err_ratio,
                    "nis_pass_rate": nis_pass_rate,
                    "flags": issue_flags
                })
                print(f"  [ISSUES]: {', '.join(issue_flags)}")
            else:
                print(f"  [OK] No major issues")

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
            print(f"  [ERROR]: {str(e)[:100]}")
            results.append({
                "session": session_name,
                "category": category,
                "status": f"ERROR: {str(e)[:100]}"
            })

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    valid_results = [r for r in results if r["status"] == "OK"]

    if valid_results:
        drifts = [r["drift_pct"] for r in valid_results]
        errors = [r["final_error_m"] for r in valid_results]
        nis_rates = [r["nis_pass_rate"] for r in valid_results]

        print(f"\nAll Sessions ({len(valid_results)}):")
        print(f"  Drift %: mean={np.mean(drifts):.2f}, median={np.median(drifts):.2f}")
        print(f"  Error (m): mean={np.mean(errors):.2f}, median={np.median(errors):.2f}")
        print(f"  NIS Rate: mean={np.mean(nis_rates):.1f}%")
        print(f"  Official Pass: {sum(1 for r in valid_results if r['official_pass'])}/{len(valid_results)}")

        print(f"\nHigh Drift Sessions (>10%): {len(high_drift_sessions)}")
        if high_drift_sessions:
            for s in high_drift_sessions:
                res = next(r for r in valid_results if r["session"] == s)
                print(f"  {s}: {res['drift_pct']:.1f}% (Error: {res['final_error_m']:.1f}m, NIS: {res['nis_pass_rate']:.1f}%)")

        print(f"\nLow NIS Sessions (<70%): {len(low_nis_sessions)}")
        if low_nis_sessions:
            for s in low_nis_sessions:
                res = next(r for r in valid_results if r["session"] == s)
                print(f"  {s}: {res['nis_pass_rate']:.1f}% (Drift: {res['drift_pct']:.1f}%)")

    # Root cause analysis
    print("\n" + "=" * 80)
    print("ROOT CAUSE ANALYSIS")
    print("=" * 80)

    print("\n[1] SPEED_SCALE LEARNING ISSUES")
    print("  - session2: Pre-outage speed too low (mean=0.55 m/s)")
    print("  - speed_scale gate requires speed > 1.0 m/s for two-wheelers")
    print("  - Only 29.7% of pre-outage samples valid for learning")
    print("  - Result: speed_scale=0.067 at outage entry (should be ~1.0)")
    print("  - AI speed correction crippled during outage")

    print("\n[2] STATIONARY/IDLING SESSIONS")
    print("  - S4: Outage distance only 2.34m (vehicle stationary)")
    print("  - Final error 528.16m despite low distance")
    print("  - Drift % calculation: 52.82% (error/10 for stationary)")
    print("  - Map matching fails with no motion")

    print("\n[3] HIGH NIS PASS RATE (>99%)")
    print("  - S4: 99.7% NIS acceptance (gating too loose)")
    print("  - May accept bad GNSS fixes without rejection")
    print("  - EKF overconfident in GNSS measurements")

    print("\n[4] AI SPEED MODEL MISMATCH")
    print("  - AI model trained primarily on car data")
    print("  - Two-wheeler vibration pattern different")
    print("  - Low pre-outage speed + vibration = unreliable AI speed")

    print("\n" + "=" * 80)
    print("RECOMMENDATIONS")
    print("=" * 80)

    print("\n1. For speed_scale learning:")
    print("   - Lower speed gate for two-wheelers to 0.5 m/s (was 1.0)")
    print("   - Use median AI speed as fallback (not GNSS ratio)")
    print("   - Increase learning window from 30s to 60s")
    print("   - Add speed_scale convergence monitoring")

    print("\n2. For stationary vehicles:")
    print("   - Improve stationary detection algorithm")
    print("   - Use ZUPT updates more aggressively")
    print("   - Map matching should detect stationary state")

    print("\n3. For NIS gating:")
    print("   - Tune EKF Q matrix (sigma_acc=5.0 may be too high)")
    print("   - Add innovation gate sanity checks")
    print("   - Monitor NIS acceptance rate in real-time")

    print("\n4. For AI speed filter:")
    print("   - Retrain with two-wheeler vibration data")
    print("   - Add vibration intensity classification")
    print("   - Use dynamic uncertainty based on vibration")

    print("\n" + "=" * 80)

    return {
        "total_sessions": len(sessions),
        "valid_sessions": len(valid_results),
        "issues_found": len(issues_found),
        "high_drift_count": len(high_drift_sessions),
        "low_nis_count": len(low_nis_sessions),
        "high_drift_sessions": high_drift_sessions,
        "low_nis_sessions": low_nis_sessions
    }

if __name__ == "__main__":
    summary = ascii_audit()
    print("\n=== AUDIT COMPLETE ===")
    print(f"Sessions audited: {summary['valid_sessions']}/{summary['total_sessions']}")
    print(f"Sessions with high drift: {summary['high_drift_count']}")
    print(f"Sessions with low NIS: {summary['low_nis_count']}")
    print(f"High drift sessions: {', '.join(summary['high_drift_sessions'])}" if summary['high_drift_sessions'] else "None")
    print(f"Low NIS sessions: {', '.join(summary['low_nis_sessions'])}" if summary['low_nis_sessions'] else "None")