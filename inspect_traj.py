import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session, load_iovnbd_session, run_car_pipeline, compute_metrics

# Let's inspect Vta28 in detail
cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}
# Let's look at what run_car_pipeline outputs
s_df, v_df = load_iovnbd_session("data/raw", cfg["driver"], cfg["session"])
metrics, res_df = run_car_pipeline(s_df, v_df, cfg["session"])

outage_mask = res_df["is_outage"].values
gt_p = res_df.loc[outage_mask, ["gt_p_x", "gt_p_y"]].values
est_p = res_df.loc[outage_mask, ["p_x", "p_y"]].values
t = res_df.loc[outage_mask, "timestamp"].values

print("Start GT:", gt_p[0], "Est:", est_p[0])
print("End GT:  ", gt_p[-1], "Est:", est_p[-1])
print("Final distance error:", np.linalg.norm(gt_p[-1] - est_p[-1]))
print("Distance along GT:", np.sum(np.linalg.norm(np.diff(gt_p, axis=0), axis=1)))
print("Distance along Est:", np.sum(np.linalg.norm(np.diff(est_p, axis=0), axis=1)))

# Let's print every 5 seconds during outage
for i in range(0, len(t), int(len(t)/12)):
    err = np.linalg.norm(gt_p[i] - est_p[i])
    print(f"t={t[i]-t[0]:.1f}s | GT: ({gt_p[i,0]:.1f}, {gt_p[i,1]:.1f}) | EST: ({est_p[i,0]:.1f}, {est_p[i,1]:.1f}) | Err: {err:.2f}m")
