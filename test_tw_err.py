import numpy as np
from training.data_loader import load_two_wheeler_session, latlon_to_enu
df = load_two_wheeler_session("data/raw/two_wheeler", "session1")
lat0, lon0 = df['phone_lat'].iloc[0], df['phone_lon'].iloc[0]
df['e_gt'], df['n_gt'], _ = latlon_to_enu(df['gt_lat'], df['gt_lon'], df['gt_alt'], lat0, lon0, 0.0)

row_135 = df.iloc[np.argmin(np.abs(df['time'] - 135.0))]
row_190 = df.iloc[np.argmin(np.abs(df['time'] - 190.0))]

print(f"GT at 135.0s: {row_135['e_gt']:.1f}, {row_135['n_gt']:.1f} | Speed: {row_135['gt_speed']:.2f}")
print(f"GT at 190.0s: {row_190['e_gt']:.1f}, {row_190['n_gt']:.1f} | Speed: {row_190['gt_speed']:.2f}")

print(f"Actual distance traveled between 135s and 190s: {np.sum(df[(df['time']>=135.0) & (df['time']<=190.0)]['gt_speed']) * 0.1:.2f} m")
print(f"Start pos: {df['e_gt'].iloc[0]}, {df['n_gt'].iloc[0]}")
