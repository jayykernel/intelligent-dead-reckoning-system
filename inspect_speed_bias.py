import numpy as np
import matplotlib.pyplot as plt
from eval.run_full_benchmark import load_iovnbd_session, preprocess_session

s_df, v_df = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta28")
synced = preprocess_session(s_df, v_df, target_dt=0.1)

gt_speed = synced["gt_speed"].values
# ai_speed is in synced, but we need the raw model output for the fusion engine
# Let's see the columns
print(synced.columns)

# AI forward speed seems to be coming from the model, maybe it's in the input data
# The fusion engine uses 'ai_speed' from the step() input.
# Let's assume it's in the session dataframe as 'ai_speed' or similar
