import os
import sys
import csv
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__))))
from eval.run_full_benchmark import evaluate_dead_reckoning_session, evaluate_edge_fog_session

def discover_sessions(base_dir):
    sessions = []
    # Car sessions
    car_dir = os.path.join(base_dir, "Categorised IOVNB Dataset")
    if os.path.isdir(car_dir):
        for driver in os.listdir(car_dir):
            driver_path = os.path.join(car_dir, driver)
            if not os.path.isdir(driver_path):
                continue
            subs = [d for d in os.listdir(driver_path) if os.path.isdir(os.path.join(driver_path, d))]
            if subs:
                for sess in subs:
                    sessions.append({"category": "car", "driver": driver, "session": sess})
            else:
                # No subdirectories, treat the driver folder as a session
                sessions.append({"category": "car", "driver": driver, "session": "session"})
    # Two-wheeler sessions
    tw_dir = os.path.join(base_dir, "two_wheeler")
    if os.path.isdir(tw_dir):
        for sess in os.listdir(tw_dir):
            sess_path = os.path.join(tw_dir, sess)
            if os.path.isdir(sess_path):
                sessions.append({"category": "two_wheeler", "driver": None, "session": sess})
    return sessions

def main():
    base_dir = "data/raw"
    all_sessions = discover_sessions(base_dir)
    print(f"Discovered {len(all_sessions)} sessions:")
    for s in all_sessions:
        print(f"  {s}")

    results = []
    print("\nStarting evaluation...")
    print("Session ID | Vehicle Category | Driver | Session | Outage Dist (m) | Final Error (m) | Drift % | Official Target (<=10%) | Stretch Target (1-2%)")
    print("-" * 120)

    for i, s in enumerate(all_sessions):
        try:
            if s["category"] == "car":
                res = evaluate_dead_reckoning_session(s)
            elif s["category"] == "two_wheeler":
                res = evaluate_dead_reckoning_session(s)
            else:
                continue
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
                "outage_dist_m": outage_dist,
                "final_error_m": final_err,
                "drift_pct": drift_pct,
                "official_pass": official_pass,
                "stretch_pass": stretch_pass
            })
        except Exception as e:
            print(f"Error processing {s}: {e}")
            # Continue to next session
            continue

        # Optional: flush after each session to avoid memory buildup
        # Not needed but we can do a small garbage collection
        if i % 10 == 0:
            import gc
            gc.collect()

    print("\nSummary:")
    print(f"Total sessions processed: {len(results)}")
    pass_official = sum(1 for r in results if r["official_pass"])
    pass_stretch = sum(1 for r in results if r["stretch_pass"])
    print(f"Sessions meeting official target (<=10% drift): {pass_official}/{len(results)}")
    print(f"Sessions meeting stretch target (1-2% drift): {pass_stretch}/{len(results)}")

    # Write results to CSV
    csv_path = "eval_all_sessions_results_final.csv"
    with open(csv_path, "w", newline='') as f:
        fieldnames = ["session_id", "category", "driver", "session", "outage_dist_m", "final_error_m", "drift_pct", "official_pass", "stretch_pass"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"\nResults written to {csv_path}")

if __name__ == "__main__":
    main()