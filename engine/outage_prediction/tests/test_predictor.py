"""
Test the OutagePredictor on the downloaded real GNSS log (pseudoranges_log_2016_06_30_21_26_07.txt).
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.outage_prediction.parse_gnss_log import parse_gnss_log
from engine.outage_prediction.outage_predictor import OutagePredictor

def run_test():
    log_path = "data/raw/pseudoranges_log_2016_06_30_21_26_07.txt"
    if not os.path.exists(log_path):
        print(f"File not found: {log_path}")
        return

    times, acc, avg_cn0, num_sats = parse_gnss_log(log_path)

    # Filter arrays to consecutive epochs (timestamp diffs ~1s)
    predictor = OutagePredictor(window_size_sec=5.0, dt=1.0)

    trust_scores = []
    for i in range(len(times)):
        cn0_val = avg_cn0[i]
        sat_val = num_sats[i]
        acc_val = acc[i]

        # A 0.0 cn0/sats means missed Status line, we shouldn't necessarily feed 0 directly unless it's a true outage.
        # But let's feed it raw for real-world robustness test.
        if np.isnan(cn0_val) or sat_val == 0:
            cn0_feed = None
            sat_feed = None
            # If 0 satellites, maybe a hard outage
            if sat_val == 0:
                sat_feed = 0
                cn0_feed = 0.0
        else:
            cn0_feed = cn0_val
            sat_feed = sat_val

        trust = predictor.update(avg_cn0=cn0_feed, sat_count=sat_feed, accuracy_m=acc_val)
        trust_scores.append(trust)

    trust_scores = np.array(trust_scores)
    times_sec = (times - times[0]) / 1000.0

    # Plotting
    os.makedirs("data/processed/phase8_eval", exist_ok=True)
    out_path = "data/processed/phase8_eval/trust_signal.png"

    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(times_sec, num_sats, 'b.-', label="Number of Satellites")
    plt.ylabel("Sat Count")
    plt.title("Real Android Raw GNSS Logger Data (2016-06-30)")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times_sec, avg_cn0, 'k.-', label="Average C/N0 (dB-Hz)")
    plt.ylabel("C/N0 (dB-Hz)")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times_sec, trust_scores, 'r.-', linewidth=2, label="Early Trust Signal [0, 1]")
    plt.ylabel("Trust Score")
    plt.xlabel("Time (s)")
    plt.ylim(-0.1, 1.1)
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()

    print(f"Validation figure saved to {out_path}")

    # Check if trust shifts before a major degradation
    degradations = np.where(num_sats < 8)[0]
    if len(degradations) > 0:
        idx = degradations[0]
        # Look 3 seconds earlier
        idx_early = max(0, idx - 3)
        print(f"Degradation (sats < 8) happens at idx {idx}, t={times_sec[idx]:.1f}s")
        print(f"Trust 3 epochs earlier (t={times_sec[idx_early]:.1f}s): {trust_scores[idx_early]:.3f}")
        print(f"Trust at degradation (t={times_sec[idx]:.1f}s): {trust_scores[idx]:.3f}")

if __name__ == "__main__":
    run_test()
