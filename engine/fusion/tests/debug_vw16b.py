import os
import sys
import pandas as pd
import numpy as np

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
s_path = os.path.join(proj_root, "data", "raw", "Categorised IOVNB Dataset", "Vw (Driver E)", "Vw16b", "S-Vw16b.csv")
try:
    df = pd.read_csv(s_path, encoding='latin1')
    print("Vw16b loaded. Shape:", df.shape)
    for i, c in enumerate(df.columns):
        print(f"{i}: '{c}'")
except Exception as e:
    print("Error:", e)

