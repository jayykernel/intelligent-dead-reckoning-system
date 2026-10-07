import numpy as np
import os
from training.data_loader import load_two_wheeler_session
from eval.run_full_benchmark import ProductionMobileFusionEngine

def main():
    raw_path = "data/raw/two_wheeler"
    session_id = 'session1'
    synced = load_two_wheeler_session(raw_path, session_id)

    dt = 0.1
    N = len(synced)
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    outage_start = min(int(N * 0.4), 3000)

    print("Pre-outage stats:")
    print("Mean speed:", np.mean(speed[:outage_start]))
    print("Max speed:", np.max(speed[:outage_start]))
    print("Min speed:", np.min(speed[:outage_start]))

    print("\nOutage stats:")
    print("Mean speed during outage:", np.mean(speed[outage_start:outage_start+600]))
    print("Max speed during outage:", np.max(speed[outage_start:outage_start+600]))

if __name__ == "__main__":
    main()
