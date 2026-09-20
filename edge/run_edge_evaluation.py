"""
edge/run_edge_evaluation.py

Validates the Edge Engine Package (Phase 12).
- Runs the synthetic FOG-grade dataset through EdgeFusionEngine.
- Computes execution rate to verify it meets the ~200Hz target.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import os
from edge.edge_engine import EdgeFusionEngine

def run_edge_evaluation():
    dataset_path = "data/synthetic_fog/s1_synthetic_fog_200hz.npz"
    if not os.path.exists(dataset_path):
        print(f"Error: Dataset {dataset_path} not found.")
        return

    print("Loading synthetic FOG dataset...")
    data = np.load(dataset_path)
    timestamps = data["time"]
    acc = data["acc"]
    gyro = data["gyro"]
    gps_pos = data["gps_pos"]
    gps_vel = data["gps_vel"]
    dt = 1.0 / data["freq"]

    # Subsample for evaluation benchmark if too large (e.g. 50,000 samples = 250s of 200Hz data)
    max_samples = 50000
    N = min(len(timestamps), max_samples)
    print(f"Dataset subset: {N} samples ({N * dt:.2f} seconds @ {data['freq']} Hz)")

    # Initialize Engine
    engine = EdgeFusionEngine(dt=dt, default_vehicle_type="car")

    # Initial state from ground truth
    p0 = gps_pos[0]
    v0 = gps_vel[0]

    # Calculate initial heading from initial velocity vector
    if np.linalg.norm(v0) > 0.5:
        heading0 = np.degrees(np.arctan2(v0[0], v0[1]))
    else:
        heading0 = 0.0

    engine.initialize_state(p0, v0, heading0)

    print("Running FOG/Edge inference pipeline...")
    out_pos = np.zeros((N, 3))

    start_wall_time = time.time()

    # Run loop
    for i in range(N):
        # Provide GNSS at ~1 Hz (every 200 samples)
        use_gnss = (i % int(data["freq"])) == 0
        p_gnss = gps_pos[i] if use_gnss else None
        v_gnss = gps_vel[i] if use_gnss else None

        res = engine.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            gnss_pos_enu=p_gnss,
            gnss_vel_enu=v_gnss,
            timestamp=timestamps[i]
        )
        out_pos[i] = res["pos"]

    end_wall_time = time.time()

    total_time = end_wall_time - start_wall_time
    throughput = N / total_time

    print(f"--- Benchmark Results ---")
    print(f"Total processing time: {total_time:.4f} seconds")
    print(f"Inference throughput:  {throughput:.2f} Hz")

    target_hz = 200.0
    if throughput >= target_hz:
        print(f"Target update rate met. ({throughput:.2f} Hz >= {target_hz} Hz)")
    else:
        print(f"Target update rate FAILED. ({throughput:.2f} Hz < {target_hz} Hz)")

    # Basic Position Plot to verify it didn't blow up
    plt.figure()
    plt.plot(gps_pos[:N, 0], gps_pos[:N, 1], 'r--', label="GNSS (1Hz)")
    plt.plot(out_pos[:N, 0], out_pos[:N, 1], 'b-', label="Edge Fusion (200Hz)")
    plt.title("Edge Engine FOG Validation")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.legend()
    plt.grid(True)
    os.makedirs("eval/plots", exist_ok=True)
    plt.savefig("eval/plots/phase12_edge_output.png")
    print("Trajectory plot saved to eval/plots/phase12_edge_output.png")

if __name__ == "__main__":
    run_edge_evaluation()
