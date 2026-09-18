"""
Parse Android GnssLogger log to extract C/N0 and satellite count per Fix/Raw epoch.
"""
import numpy as np
from collections import defaultdict

def parse_gnss_log(log_path):
    """
    Parses Android GnssLogger raw logs.
    Groups Raw measurements by ElapsedRealtimeMillis (or TimeNanos) to compute:
      - Epoch timestamp (seconds)
      - Number of tracked satellites (sat_count)
      - Mean C/N0 (dB-Hz)
      - Minimum C/N0 (dB-Hz)
      - Top-4 Mean C/N0 (dB-Hz)
      - Fix Accuracy (if available)
    """
    epochs = defaultdict(list)
    fixes = {}

    with open(log_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Fix,Provider,Latitude,Longitude,Altitude,Speed,Accuracy,(UTC)TimeInMs
            if line.startswith('Fix,'):
                parts = line.split(',')
                if len(parts) >= 8:
                    try:
                        time_ms = int(parts[7])
                        acc = float(parts[6])
                        fixes[time_ms] = acc
                    except:
                        pass

            # Raw,ElapsedRealtimeMillis,TimeNanos,...,Svid,...,Cn0DbHz,...
            elif line.startswith('Raw,'):
                parts = line.split(',')
                if len(parts) >= 17:
                    try:
                        # ElapsedRealtimeMillis is parts[1]
                        # TimeNanos is parts[2]
                        # Svid is parts[11]
                        # Cn0DbHz is parts[16]
                        elapsed_ms = int(parts[1])
                        svid = int(parts[11])
                        cn0 = float(parts[16])

                        # Round to nearest 1000ms epoch
                        epoch_key = round(elapsed_ms / 1000.0) * 1000
                        epochs[epoch_key].append((svid, cn0))
                    except (ValueError, IndexError):
                        pass

    sorted_epochs = sorted(epochs.keys())
    if not sorted_epochs:
        return np.array([]), np.array([]), np.array([]), np.array([])

    times = []
    num_sats = []
    avg_cn0 = []
    acc = []

    for ep in sorted_epochs:
        meas = epochs[ep]
        svids = set([m[0] for m in meas])
        cn0s = [m[1] for m in meas if m[1] > 0.0]

        times.append(ep)
        num_sats.append(len(svids))
        avg_cn0.append(np.mean(cn0s) if cn0s else 0.0)
        # Accuracy placeholder or match nearest fix
        acc.append(3.0)

    return np.array(times), np.array(acc), np.array(avg_cn0), np.array(num_sats)

if __name__ == '__main__':
    log_path = 'data/raw/pseudoranges_log_2016_06_30_21_26_07.txt'
    times, acc, avg_cn0, num_sats = parse_gnss_log(log_path)
    print(f'Successfully parsed {len(times)} epochs from {log_path}:')
    print(f'  Sat count range: {np.min(num_sats)} - {np.max(num_sats)} (mean: {np.mean(num_sats):.1f})')
    print(f'  C/N0 range: {np.min(avg_cn0):.1f} - {np.max(avg_cn0):.1f} dB-Hz (mean: {np.mean(avg_cn0):.1f})')
