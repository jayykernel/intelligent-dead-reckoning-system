import sys
import numpy as np
sys.path.insert(0, r"C:\dev\dead reckoning proto")

from eval.run_full_benchmark import evaluate_dead_reckoning_session
from training.data_loader import load_iovnbd_session, preprocess_session

s_df, v_df = load_iovnbd_session(r"data\raw\Categorised IOVNB Dataset", "Vta (Driver E)", "Vta27")
df = preprocess_session(s_df, v_df, target_dt=0.1)

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})
start = res["outage_start"]
end = res["outage_end"]
results = res["results"]

print("idx | time | EKF yaw | mag used | mag yaw | gnss | map snapped | AI spd | scale | EKF spd")
for i in range(start - 20, min(start + 200, end)):
    t = df['time'].iloc[i]
    r = results[i]
    yaw = r['euler_deg'][2]
    
    ekf_spd = np.linalg.norm(r['vel'])
    ai_spd = r['ai_speed']
    scale = r['speed_scale']
    
    # Unfortunately mag_yaw and map_snapped are not preserved in the `results` list directly except as 'mag_disturbed'
    mode = r['mode']
    
    # We can rebuild loop logic inside trace_vta27 if needed, but 'mag_disturbed' is available.
    
    if i % 10 == 0 or t > 97.0 and t < 103.0:
        print(f"{i:4d} | {t:5.1f}s | {yaw:6.1f} | mag_dist={r['mag_disturbed']} | {mode} | spd={ekf_spd:.1f} | AI={ai_spd*scale:.1f}")

