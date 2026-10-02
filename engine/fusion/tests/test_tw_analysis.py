import numpy as np
from training.data_loader import load_two_wheeler_session

import pandas as pd
import numpy as np
from training.data_loader import load_two_wheeler_session
from engine.ai_filters.speed_filter import SpeedFilter

synced = load_two_wheeler_session("data/raw/two_wheeler", "session1")
filter_model = SpeedFilter("training/models/speed_filter.tflite")

imu = synced[["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]].values
speeds_ai = filter_model.predict_sequence(imu)
gt_speeds = synced["gt_speed"].values

outage_start = min(int(len(synced) * 0.4), 3000)
outage_end = outage_start + 600

print(f"Pre-outage mean: GT={gt_speeds[:outage_start].mean():.2f} m/s, AI={speeds_ai[:outage_start].mean():.2f} m/s")
print(f"Outage mean: GT={gt_speeds[outage_start:outage_end].mean():.2f} m/s, AI={speeds_ai[outage_start:outage_end].mean():.2f} m/s")

# Let's inspect ratio
ratio = gt_speeds[:outage_start].mean() / max(1e-3, speeds_ai[:outage_start].mean())
print(f"Scale factor (GT / AI): {ratio:.3f}")


