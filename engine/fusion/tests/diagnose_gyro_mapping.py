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

print(f"{'Session':<10} | {'Acc Means (X, Y, Z)':<25} | {'Max Gyro Std':<15} | {'Dominant Gyro col with GT Turn'}")
print("-" * 80)

for driver, sess in test_sessions:
    s_files = glob.glob(f'data/raw/**/{driver}/{sess}/S-*.csv', recursive=True) or glob.glob(f'data/raw/**/{sess}/S-*.csv', recursive=True)
    v_files = glob.glob(f'data/raw/**/{driver}/{sess}/[Vv]-*.csv', recursive=True) or glob.glob(f'data/raw/**/{sess}/[Vv]-*.csv', recursive=True)

    if not s_files or not v_files:
        continue

    s_df = pd.read_csv(s_files[0], encoding='latin1')
    v_df = pd.read_csv(v_files[0], encoding='latin1')
    s_df.columns = [c.strip() for c in s_df.columns]
    v_df.columns = [c.strip() for c in v_df.columns]

    s_time = (s_df['TIME SINCE START (ms)'] - s_df['TIME SINCE START (ms)'].iloc[0]) / 1000.0
    v_time = v_df['Time Since Start of Day (seconds)'] - v_df['Time Since Start of Day (seconds)'].iloc[0]
    t_grid = np.arange(0, min(s_time.iloc[-1], v_time.iloc[-1]), 0.1)

    acc_x = np.interp(t_grid, s_time, s_df.filter(like='ACCELEROMETER X').iloc[:, 0])
    acc_y = np.interp(t_grid, s_time, s_df.filter(like='ACCELEROMETER Y').iloc[:, 0])
    acc_z = np.interp(t_grid, s_time, s_df.filter(like='ACCELEROMETER Z').iloc[:, 0])

    acc_means = f"({acc_x.mean():.1f}, {acc_y.mean():.1f}, {acc_z.mean():.1f})"

    # Ground truth yaw rate
    unwrapped_h = np.unwrap(np.radians(v_df['Heading (degrees)']))
    gt_yaw_rate = np.diff(np.interp(t_grid, v_time, unwrapped_h)) / 0.1
    gt_yaw_rate = np.append(gt_yaw_rate, gt_yaw_rate[-1])

    corrs = {}
    for c in [x for x in s_df.columns if 'GYROSCOPE' in x]:
        val = np.interp(t_grid, s_time, s_df[c])
        c_short = c.split()[1]
        corrs[c_short] = np.corrcoef(val, gt_yaw_rate)[0, 1]

    best_c = max(corrs.items(), key=lambda x: abs(x[1]))
    all_c_str = ", ".join([f"{k}:{v:+.2f}" for k, v in corrs.items()])
    print(f"{sess:<10} | {acc_means:<25} | {all_c_str}")
