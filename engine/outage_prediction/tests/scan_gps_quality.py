"""
Search all IO-VNBD sessions for GPS outages, satellite drops, accuracy jumps.
"""
import os
import glob
import pandas as pd
import numpy as np

def scan_sessions():
    s_files = glob.glob("data/raw/**/S-*.csv", recursive=True) + glob.glob("data/raw/**/s-*.csv", recursive=True)
    print(f"Scanning {len(s_files)} sessions for GPS anomalies/outages...")

    for f in s_files:
        try:
            df = pd.read_csv(f, encoding='latin1')
            df.columns = [c.strip() for c in df.columns]

            sat_col = None
            for c in df.columns:
                if 'SATELLITES' in c:
                    sat_col = c
                    break

            acc_col = None
            for c in df.columns:
                if 'ACCURACY' in c:
                    acc_col = c
                    break

            lat_col = None
            for c in df.columns:
                if 'LATITUDE' in c:
                    lat_col = c
                    break

            if sat_col and acc_col and lat_col:
                sats = df[sat_col].values
                acc = df[acc_col].values
                # Just check a subset if sats is too big or check if NaNs exist
                if isinstance(sats[0], str): continue # Skip if bad data

                min_sats = np.min(sats)
                max_acc = np.max(acc)
                sat_drops = np.where(np.diff(sats) < -3)[0]

                if min_sats <= 5 or max_acc > 30.0 or len(sat_drops) > 5:
                    session_name = os.path.basename(os.path.dirname(f))
                    print(f"Session {session_name} ({f}):")
                    print(f"  Min sats: {min_sats}, Max sats: {np.max(sats)}")
                    print(f"  Min acc: {np.min(acc):.1f}m, Max acc: {max_acc:.1f}m")
                    print(f"  Sat drop events (diff < -3): {len(sat_drops)}")
        except Exception as e:
            # print(f"Error {f}: {e}")
            pass

if __name__ == '__main__':
    scan_sessions()
