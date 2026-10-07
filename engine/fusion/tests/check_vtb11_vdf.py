import os
import numpy as np
import pandas as pd
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from training.data_loader import load_iovnbd_session

raw_root = 'data/raw'
driver = 'Vtb (Driver E)'
session = 'Vtb11'
s_df, v_df = load_iovnbd_session(raw_root, driver, session)
print('v_df columns:', v_df.columns.tolist())
print('v_df head:')
print(v_df.head())
print('v_df shape:', v_df.shape)