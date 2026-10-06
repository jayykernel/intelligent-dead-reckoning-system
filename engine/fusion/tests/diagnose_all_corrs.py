import glob
import os
import pandas as pd
import numpy as np

test_sessions = [
    ('Vta (Driver E)', 'Vta26'),
    ('Vta (Driver E)', 'Vta27'),
    ('Vta (Driver E)', 'Vta28'),
    ('Vta (Driver E)', 'Vta29'),
    ('Vta (Driver E)', 'Vta30'),
    ('Vtb (Driver D)', 'Vtb11'),
    ('Vtb (Driver D)', 'Vtb12'),
    ('Vw (Driver A)', 'Vw15'),
    ('Vw (Driver A)', 'Vw16a'),
    ('Vw (Driver A)', 'Vw16b'),
    ('Vw (Driver A)', 'Vw17'),
    ('V-Vfa (Driver C)', 'V-Vfa02'),
    ('S (Driver B)', 'S4')
]

for driver, sess in test_sessions:
    s_files = glob.glob(f'data/raw/**/{driver}/{sess}/S-*.csv', recursive=True) or glob.glob(f'data/raw/**/{sess}/S-*.csv', recursive=True)
    v_files = glob.glob(f'data/raw/**/{driver}/{sess}/[Vv]-*.csv', recursive=True) or glob.glob(f'data/raw/**/{sess}/[Vv]-*.csv', recursive=True)

    if not s_files or not v_files:
        print(f'Missing files for {sess}')
        continue

    s_df = pd.read_csv(s_files[0], encoding='latin1')
    v_df = pd.read_csv(v_files[0], encoding='latin1')
    s_df.columns = [c.strip() for c in s_df.columns]
    v_df.columns = [c.strip() for c in v_df.columns]

    s_time = (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0
    v_time = v_df['Time Since Start of Day (seconds)'] - v_df['Time Since Start of Day (seconds)'].iloc[0]
    t_grid = np.arange(0, min(s_time.iloc[-1], v_time.iloc[-1]), 0.1)

    if 'Yaw Rate (deg/sec)' in v_df.columns:
        can_yaw_rate = np.radians(np.interp(t_grid, v_time, v_df['Yaw Rate (deg/sec)']))
    else:
        unwrapped_h = np.unwrap(np.radians(v_df['Heading (degrees)']))
        can_yaw_rate = np.diff(np.interp(t_grid, v_time, unwrapped_h)) / 0.1
        can_yaw_rate = np.append(can_yaw_rate, can_yaw_rate[-1])

    corrs = {}
    for c in [x for x in s_df.columns if 'GYROSCOPE' in x]:
        val = np.interp(t_grid, s_time, s_df[c])
        c_short = c.split()[1] # Yaw, Pitch, Roll
        corrs[c_short] = np.corrcoef(val, can_yaw_rate)[0, 1]

    y_c = corrs.get('Yaw', 0)
    p_c = corrs.get('Pitch', 0)
    r_c = corrs.get('Roll', 0)
    print(f'{sess:10s} | Yaw_col: {y_c:+.3f} | Pitch_col: {p_c:+.3f} | Roll_col: {r_c:+.3f}')
