"""
Search IO-VNBD sessions for GNSS signal quality patterns.
"""
import os
import glob
import pandas as pd
import numpy as np

def parse_sat_str(s):
    try:
        # Format "27 / 28" -> usable/total
        parts = str(s).split('/')
        return int(parts[0].strip()) if len(parts) > 0 else 0
    except:
        return 0

def scan_sessions():
    s_files = glob.glob("data/raw/**/S-*.csv", recursive=True) + glob.glob("data/raw/**/s-*.csv", recursive=True)
    print(f"Scanning {len(s_files)} sessions for GNSS signal patterns...")

    for f in s_files:
        try:
            df = pd.read_csv(f, encoding='latin1')
            df.columns = [c.strip() for c in df.columns]

            sat_col = 'GPS SATELLITES IN RANGE'
            acc_col = 'GPS ACCURACY (m)'

            if sat_col in df.columns:
                # Parse "27 / 28" -> 27
                sats = df[sat_col].apply(parse_sat_str).values
                acc = pd.to_numeric(df[acc_col], errors='coerce').fillna(100.0).values

                # Look for sudden drop in sats or jump in accuracy
                diff_sats = np.diff(sats)

                # Drop of >= 5 satellites
                drop_events = np.where(diff_sats <= -5)[0]

                # Spike in accuracy
                spike_events = np.where(acc > 20.0)[0]

                if len(drop_events) > 0 or len(spike_events) > 0:
                    session_name = os.path.basename(os.path.dirname(f))
                    print(f"Session {session_name} ({f}):")
                    print(f"  Sat drop events: {len(drop_events)}")
                    print(f"  Accuracy spike events: {len(spike_events)}")
        except Exception as e:
            pass

if __name__ == '__main__':
    scan_sessions()
