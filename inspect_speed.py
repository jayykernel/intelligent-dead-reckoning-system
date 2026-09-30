import numpy as np
from eval.run_full_benchmark import load_iovnbd_session, preprocess_session

s_df, v_df = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta28")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

gt_speed = synced["gt_speed"].values
N = len(synced)
outage_start = min(int(N * 0.4), 3000)
outage_end = outage_start + 600

print(f"Vta28 Outage indices: {outage_start} to {outage_end}")
print(f"Mean GT speed before outage: {np.mean(gt_speed[:outage_start]):.2f} m/s")
print(f"Mean GT speed during outage: {np.mean(gt_speed[outage_start:outage_end]):.2f} m/s")
print(f"Max GT speed during outage:  {np.max(gt_speed[outage_start:outage_end]):.2f} m/s")
print(f"GT distance during outage:   {np.sum(gt_speed[outage_start:outage_end])*0.1:.2f} m")

# Let's check Vta29
s_df2, v_df2 = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta29")
synced2 = preprocess_session(s_df2, v_df2, target_dt=0.1)
gt_speed2 = synced2["gt_speed"].values
N2 = len(synced2)
outage_start2 = min(int(N2 * 0.4), 3000)
outage_end2 = outage_start2 + 600
print(f"\nVta29 Outage indices: {outage_start2} to {outage_end2}")
print(f"Mean GT speed before outage: {np.mean(gt_speed2[:outage_start2]):.2f} m/s")
print(f"Mean GT speed during outage: {np.mean(gt_speed2[outage_start2:outage_end2]):.2f} m/s")
print(f"Max GT speed during outage:  {np.max(gt_speed2[outage_start2:outage_end2]):.2f} m/s")
print(f"GT distance during outage:   {np.sum(gt_speed2[outage_start2:outage_end2])*0.1:.2f} m")
