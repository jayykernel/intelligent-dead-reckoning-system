import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__))))
from eval.run_full_benchmark import evaluate_dead_reckoning_session

# List all car sessions from the categorized IOVNB dataset
sessions = [
    # Driver A (S)
    {"category": "car", "driver": "S (Driver A)", "session": "S1"},
    {"category": "car", "driver": "S (Driver A)", "session": "S2"},
    {"category": "car", "driver": "S (Driver A)", "session": "S3a"},
    {"category": "car", "driver": "S (Driver A)", "session": "S3b"},
    {"category": "car", "driver": "S (Driver A)", "session": "S3c"},
    {"category": "car", "driver": "S (Driver A)", "session": "S4"},
    # Driver B (M)
    {"category": "car", "driver": "M (Driver B)", "session": "M1"},
    {"category": "car", "driver": "M (Driver B)", "session": "M2"},
    {"category": "car", "driver": "M (Driver B)", "session": "M3"},
    {"category": "car", "driver": "M (Driver B)", "session": "M4"},
    # Driver E (Vf)
    {"category": "car", "driver": "Vf (Driver E)", "session": "Vf1"},
    {"category": "car", "driver": "Vf (Driver E)", "session": "Vf2"},
    # Driver E (Vta)
    {"category": "car", "driver": "Vta (Driver E)", "session": "Vta26"},
    # Driver E (Vtb)
    {"category": "car", "driver": "Vtb (Driver E)", "session": "Vtb1"},
    {"category": "car", "driver": "Vtb (Driver E)", "session": "Vtb2"},
    # Driver E (Vw)
    {"category": "car", "driver": "Vw (Driver E)", "session": "Vw1"},
    {"category": "car", "driver": "Vw (Driver E)", "session": "Vw2"},
    # Driver D (Y)
    {"category": "car", "driver": "Y (Driver D)", "session": "Y1"},
    {"category": "car", "driver": "Y (Driver D)", "session": "Y2"},
]

print("Session ID | Vehicle Category | Data Source / Platform | Outage Dist (m) | Final Error (m) | Drift % | Official Target (<=10%) | Stretch Target (1-2%)")
print("-" * 120)
results = []
for s in sessions:
    try:
        res = evaluate_dead_reckoning_session(s)
        # Extract needed fields
        category = s["category"]
        driver = s["driver"]
        session_name = s["session"]
        # Use a combined identifier like "S1" or "M1"
        session_id = f"{session_name}"
        # The evaluate function returns outage_dist_m, final_error_m, drift_pct
        outage_dist = res["outage_dist_m"]
        final_err = res["final_error_m"]
        drift_pct = res["drift_pct"]
        official_pass = drift_pct <= 10.0
        stretch_pass = drift_pct <= 2.0
        print(f"{session_id:10} | {category:4} | {driver:20} | {outage_dist:13.2f} | {final_err:13.2f} | {drift_pct:6.2f}% | {'PASS' if official_pass else 'FAIL':11} | {'PASS' if stretch_pass else 'FAIL':12}")
        results.append({
            "session_id": session_id,
            "driver": driver,
            "outage_dist": outage_dist,
            "final_error": final_err,
            "drift_pct": drift_pct,
            "official_pass": official_pass,
            "stretch_pass": stretch_pass
        })
    except Exception as e:
        print(f"Error processing {s}: {e}")

# Summary
print("\nSummary:")
print(f"Total sessions processed: {len(results)}")
pass_official = sum(1 for r in results if r["official_pass"])
pass_stretch = sum(1 for r in results if r["stretch_pass"])
print(f"Sessions meeting official target (<=10% drift): {pass_official}/{len(results)}")
print(f"Sessions meeting stretch target (1-2% drift): {pass_stretch}/{len(results)}")