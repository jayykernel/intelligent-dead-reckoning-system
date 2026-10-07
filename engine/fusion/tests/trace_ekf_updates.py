import sys
import numpy as np
sys.path.insert(0, r"C:\dev\dead reckoning proto")

from eval.run_full_benchmark import evaluate_dead_reckoning_session, build_gt_road_network
from training.data_loader import load_iovnbd_session, preprocess_session

# Let's run evaluation and check the EKF nis_history or custom instrumenting
from engine.fusion.ekf import ErrorStateEKF

s_df, v_df = load_iovnbd_session(r"data\raw\Categorised IOVNB Dataset", "Vta (Driver E)", "Vta27")
df = preprocess_session(s_df, v_df, target_dt=0.1)

res = evaluate_dead_reckoning_session({"category": "car", "driver": "Vta (Driver E)", "session": "Vta27"})
start = res["outage_start"]
end = res["outage_end"]
results = res["results"]

print(f"Total steps in results: {len(results)}")
print(f"Outage range: {start} ({df['time'].iloc[start]:.1f}s) to {end} ({df['time'].iloc[end]:.1f}s)")

# Let's inspect raw gyros in vehicle frame vs phone frame
from engine.calibration.calibrator import CalibrationEngine
calib = CalibrationEngine()
calib.calibrate_from_session(
    acc=df[['acc_x', 'acc_y', 'acc_z']].values[:300],
    gyro=df[['gyro_x', 'gyro_y', 'gyro_z']].values[:300],
    speed=df['gt_speed'].values[:300],
    dt=0.1
)

acc_veh, gyro_veh = calib.apply(df[['acc_x', 'acc_y', 'acc_z']].values, df[['gyro_x', 'gyro_y', 'gyro_z']].values)

print("\nGyro Z (rad/s) and estimated yaw rate around 97-105s:")
for i in range(start + 50, start + 150, 5):
    t = df['time'].iloc[i]
    print(f"{i} | {t:.1f}s | gyro_veh_z = {gyro_veh[i, 2]:.4f} rad/s ({np.degrees(gyro_veh[i, 2]):.2f} deg/s) | raw gyro_z = {df['gyro_z'].iloc[i]:.4f}")

