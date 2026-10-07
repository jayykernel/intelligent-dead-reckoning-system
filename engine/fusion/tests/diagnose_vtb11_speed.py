import sys, os, numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from training.data_loader import load_iovnbd_session, preprocess_session
from training.dataset_splits import TEST_SESSIONS

driver = next(d for d, s in TEST_SESSIONS if s == "Vtb11")
s_df, v_df = load_iovnbd_session("data/raw", driver, "Vtb11")
synced = preprocess_session(s_df, v_df, target_dt=0.1)
speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)

print(f"Max speed: {np.max(speed)}, Min speed: {np.min(speed)}")
ds = np.gradient(speed, 0.1)
print(f"Max accel: {np.max(ds):.2f}, Min accel (brake): {np.min(ds):.2f}")
print(f"Frames with accel > 0.5: {np.sum(ds > 0.5)}")
print(f"Frames with accel > 0.2: {np.sum(ds > 0.2)}")
