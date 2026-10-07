import sys
import numpy as np
sys.path.insert(0, r"C:\dev\dead reckoning proto")
from eval.run_full_benchmark import evaluate_dead_reckoning_session

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})
start = res["outage_start"]
end = res["outage_end"]

# Let's inspect raw gyro from dataset
from training.data_loader import load_car_dataset
df = load_car_dataset("Vta (Driver E)", "Vta27")
print("Dataset length:", len(df))

# Let's check updates in outage
# We can't access fusion directly from res, but we can look at res['results']
for i in range(start + 40, start + 60):
    t = i * 0.1
    r = res["results"][i]
    print(f"t={t:.1f} yaw={r['euler_deg'][2]:.1f}")

