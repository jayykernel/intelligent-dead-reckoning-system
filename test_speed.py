from engine.ai_filters.speed_filter import SpeedFilter
from training.data_loader import load_two_wheeler_session
import numpy as np

sf = SpeedFilter("training/models/speed_filter_tcn.tflite")
df = load_two_wheeler_session("data/raw/two_wheeler", "session1")

acc = df[['acc_x', 'acc_y', 'acc_z']].values
gyro = df[['gyro_x', 'gyro_y', 'gyro_z']].values

preds = []
for i in range(len(df)):
    speed, var = sf.process_sample(acc[i], gyro[i])
    if speed is not None:
        preds.append((df['time'].iloc[i], speed, df['gt_speed'].iloc[i]))

import pandas as pd
p_df = pd.DataFrame(preds, columns=['time', 'ai_speed', 'gt_speed'])
outage_df = p_df[(p_df['time'] >= 134.3) & (p_df['time'] <= 194.3)]

print(f"Overall Mean AI speed: {p_df['ai_speed'].mean():.2f}, GT: {p_df['gt_speed'].mean():.2f}")
print(f"Outage Mean AI speed:  {outage_df['ai_speed'].mean():.2f}, GT: {outage_df['gt_speed'].mean():.2f}")
print(f"Outage Max AI speed:   {outage_df['ai_speed'].max():.2f}")
