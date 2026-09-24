import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__))))

# We'll copy the necessary functions from run_full_benchmark.py to avoid modifying the original
# but we can import them if they are accessible.
from eval.run_full_benchmark import (
    evaluate_dead_reckoning_session,
    evaluate_edge_fog_session,
    ProductionMobileFusionEngine,
    build_gt_road_network
)
from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from engine.map_matching.hmm_matcher import HMMMapMatcher
import numpy as np

def list_car_sessions():
    """List all car sessions from the categorized IOVNB dataset."""
    base = "data/raw/Categorised IOVNB Dataset"
    sessions = []
    if not os.path.isdir(base):
        return sessions
    for driver in os.listdir(base):
        driver_path = os.path.join(base, driver)
        if not os.path.isdir(driver_path):
            continue
        # Look for session subdirectories
        subs = [d for d in os.listdir(driver_path) if os.path.isdir(os.path.join(driver_path, d))]
        if subs:
            for sess in subs:
                sessions.append({"category": "car", "driver": driver, "session": sess})
        else:
            # No subdirectories, treat the driver folder as a session named "session"
            sessions.append({"category": "car", "driver": driver, "session": "session"})
    return sessions

def list_two_wheeler_sessions():
    base = "data/raw/two_wheeler"
    sessions = []
    if not os.path.isdir(base):
        return sessions
    for sess in os.listdir(base):
        sess_path = os.path.join(base, sess)
        if os.path.isdir(sess_path):
            sessions.append({"category": "two_wheeler", "driver": None, "session": sess})
    return sessions

def evaluate_session_custom(session_config):
    """Evaluate a single session using the same logic as evaluate_dead_reckoning_session but with optional parameters."""
    # We'll reuse the evaluate_dead_reckoning_session function from run_full_benchmark.py
    # but we can override some parameters if needed.
    # For now, just call the existing function.
    return evaluate_dead_reckoning_session(session_config)

def main():
    car_sessions = list_car_sessions()
    tw_sessions = list_two_wheeler_sessions()
    all_sessions = car_sessions + tw_sessions
    print(f"Found {len(car_sessions)} car sessions and {len(tw_sessions)} two-wheeler sessions.")

    # Print header
    print("\nSession ID | Vehicle Category | Driver | Session | Outage Dist (m) | Final Error (m) | Drift % | Official Target (<=10%) | Stretch Target (1-2%)")
    print("-" * 120)

    results = []
    for s in all_sessions:
        try:
            # For two-wheeler, we might want to disable AI to avoid loading models (if not present)
            # We can temporarily modify the session config to pass enable_ai=False to the fusion engine.
            # However, evaluate_dead_reckoning_session does not expose that.
            # Instead, we can create a custom evaluation that uses ProductionMobileFusionEngine with enable_ai=False.
            # But for simplicity, we'll just use the existing function and hope the models are not required (they are optional).
            # If the model files are missing, the AI corrector will be disabled anyway.
            res = evaluate_dead_reckoning_session(s)
            session_id = s["session"]
            outage_dist = res["outage_dist_m"]
            final_err = res["final_error_m"]
            drift_pct = res["drift_pct"]
            official_pass = drift_pct <= 10.0
            stretch_pass = drift_pct <= 2.0
            print(f"{session_id:10} | {s['category']:6} | {s['driver'] if s['driver'] else 'None':12} | {s['session']:10} | {outage_dist:13.2f} | {final_err:13.2f} | {drift_pct:6.2f}% | {'PASS' if official_pass else 'FAIL':11} | {'PASS' if stretch_pass else 'FAIL':12}")
            results.append({
                "session_id": session_id,
                "category": s["category"],
                "driver": s["driver"],
                "session": s["session"],
                "outage_dist": outage_dist,
                "final_error": final_err,
                "drift_pct": drift_pct,
                "official_pass": official_pass,
                "stretch_pass": stretch_pass
            })
        except Exception as e:
            print(f"Error processing {s}: {e}")
            import traceback
            traceback.print_exc()

    print("\nSummary:")
    print(f"Total sessions processed: {len(results)}")
    pass_official = sum(1 for r in results if r["official_pass"])
    pass_stretch = sum(1 for r in results if r["stretch_pass"])
    print(f"Sessions meeting official target (<=10% drift): {pass_official}/{len(results)}")
    print(f"Sessions meeting stretch target (1-2% drift): {pass_stretch}/{len(results)}")

    # Optionally, write results to a CSV file
    import csv
    with open("eval_all_sessions_results.csv", "w", newline='') as f:
        fieldnames = ["session_id", "category", "driver", "session", "outage_dist_m", "final_error_m", "drift_pct", "official_pass", "stretch_pass"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print("\nResults written to eval_all_sessions_results.csv")

if __name__ == "__main__":
    main()