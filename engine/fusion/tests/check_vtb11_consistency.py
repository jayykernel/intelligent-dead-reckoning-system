import os
import numpy as np
import pandas as pd
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine
from engine.fusion.ai_corrector import AICorrectionModule

raw_root = 'data/raw'
driver = 'Vtb (Driver E)'
session = 'Vtb11'
s_df, v_df = load_iovnbd_session(raw_root, driver, session)
synced = preprocess_session(s_df, v_df, target_dt=0.1)
acc = synced[['acc_x','acc_y','acc_z']].values
gyro = synced[['gyro_x','gyro_y','gyro_z']].values
speed = synced['gt_speed'].values
# Calibrate
cal = CalibrationEngine()
cal.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=0.1)
# AI corrector
ai = AICorrectionModule()
# Pre-outage: first 137 samples (since outage_start=137 from diagnose_vtb11.py)
ai_speeds = []
gnss_speeds = []
for i in range(137):
    ai_speed, sigma_ai, q_scale = ai.process_imu_sample(acc[i], gyro[i])
    if ai_speed is not None:
        # GNSS velocity from v_df
        vx = v_df.iloc[i]['vx'] if 'vx' in v_df.columns else 0.0
        vy = v_df.iloc[i]['vy'] if 'vy' in v_df.columns else 0.0
        gnss_speed = np.linalg.norm([vx, vy])
        ai_speeds.append(ai_speed)
        gnss_speeds.append(gnss_speed)
ai_speeds = np.array(ai_speeds)
gnss_speeds = np.array(gnss_speeds)
print('Number of samples:', len(ai_speeds))
if len(ai_speeds) > 0:
    corr = np.corrcoef(ai_speeds, gnss_speeds)[0,1]
    print('Correlation between AI speed and GNSS speed:', corr)
    print('Mean AI speed:', np.mean(ai_speeds))
    print('Mean GNSS speed:', np.mean(gnss_speeds))
    print('Std AI speed:', np.std(ai_speeds))
    print('Std GNSS speed:', np.std(gnss_speeds))
    # Also check if AI speed is stuck at 1.0 or default?
    print('AI speed min:', np.min(ai_speeds))
    print('AI speed max:', np.max(ai_speeds))
    print('AI speed mean:', np.mean(ai_speeds))
    print('AI speed median:', np.median(ai_speeds))