"""
engine/run_ai_speed_filter_baseline.py

Phase 3 - Evaluates the AI Speed & Vibration Filter against the Phase 2 baseline.

Integrates the predicted forward velocity with gyro attitude to compute dead reckoning trajectory,
and compares position error / drift percentage directly against:
1. Ground Truth trajectory
2. Phase 2 Classical Strapdown INS baseline
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Make engine package importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from engine.strapdown import StrapdownINS
from engine.ai_filters.speed_filter import SpeedFilter
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu


def run(
    driver: str = "S (Driver A)",
    session: str = "S1",
    model_path: str = "training/models/speed_filter.tflite",
    raw_root: str = "data/raw",
    out_root: str = "data/processed",
    window_sec: float = 60.0,
):
    print(f"[1/4] Loading session {driver} / {session}...")
    s_df, v_df = load_iovnbd_session(raw_root, driver, session)
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    dt = 0.1
    if window_sec is not None:
        N = int(window_sec / dt)
        N = min(N, len(synced))
        synced = synced.iloc[:N]

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    imu_6d = np.hstack([acc, gyro])
    N = len(synced)

    # Initial state from ground truth
    gt_speed_0 = synced["gt_speed"].iloc[0]
    gt_heading_0 = synced["gt_heading"].iloc[0]
    heading_rad_0 = np.radians(gt_heading_0)
    v0 = np.array([gt_speed_0 * np.sin(heading_rad_0), gt_speed_0 * np.cos(heading_rad_0), 0.0])
    p0 = np.zeros(3)

    print("[2/4] Running Phase 2 Classical Strapdown INS baseline...")
    ins = StrapdownINS()
    ins.initialize_from_gravity_and_heading(p0, v0, acc[0], gt_heading_0)
    strapdown_pos, strapdown_vel, strapdown_quat = ins.run_trajectory(acc, gyro, dt, p0, v0)

    print(f"[3/4] Running Phase 3 AI Speed Filter with model {model_path}...")
    speed_filter = SpeedFilter(model_path)
    pred_speeds = speed_filter.predict_sequence(imu_6d)

    # Dead reckoning using AI estimated speed + gyro-integrated orientation
    # Attitude from strapdown quaternion history
    ai_pos = np.zeros((N, 3))
    ai_vel = np.zeros((N, 3))
    ai_pos[0] = p0
    ai_vel[0] = v0

    for i in range(1, N):
        q = strapdown_quat[i]
        # Body frame +Y is forward in phone or +X depending on alignment.
        # From strapdown DCM: R_b^n transforms body to ENU
        # For heading from quaternion: yaw angle Psi
        # In ENU: heading clockwise from North is psi_nav = atan2(R[0,1], R[1,1]) or direct heading
        # Using DCM forward vector:
        qw, qx, qy, qz = q
        # R_b^n matrix components for forward direction
        # In phone frame, specific forward axis or heading:
        # Yaw angle psi:
        psi = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
        # In navigation frame (ENU): East = v * sin(heading), North = v * cos(heading)
        # Using initial alignment:
        heading = gt_heading_0 + np.degrees(np.unwrap([0.0, psi - np.arctan2(2.0 * (strapdown_quat[0, 0] * strapdown_quat[0, 3] + strapdown_quat[0, 1] * strapdown_quat[0, 2]), 1.0 - 2.0 * (strapdown_quat[0, 2]**2 + strapdown_quat[0, 3]**2))])[1])
        h_rad = np.radians(heading)

        speed_i = pred_speeds[i]
        ai_vel[i, 0] = speed_i * np.sin(h_rad)
        ai_vel[i, 1] = speed_i * np.cos(h_rad)
        ai_vel[i, 2] = 0.0

        ai_pos[i] = ai_pos[i - 1] + ai_vel[i] * dt

    # Ground truth ENU
    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, _ = latlon_to_enu(
        synced["gt_lat"].values,
        synced["gt_lon"].values,
        synced["gt_alt"].values,
        lat0, lon0, alt0
    )

    # Compute metrics
    gt_dist = float(np.sum(np.sqrt(np.diff(e_gt) ** 2 + np.diff(n_gt) ** 2)))

    strapdown_err = np.sqrt((strapdown_pos[-1, 0] - e_gt[-1])**2 + (strapdown_pos[-1, 1] - n_gt[-1])**2)
    strapdown_drift_pct = (strapdown_err / gt_dist) * 100.0 if gt_dist > 0 else np.nan

    ai_err = np.sqrt((ai_pos[-1, 0] - e_gt[-1])**2 + (ai_pos[-1, 1] - n_gt[-1])**2)
    ai_drift_pct = (ai_err / gt_dist) * 100.0 if gt_dist > 0 else np.nan

    # Speed metrics
    gt_speeds = synced["gt_speed"].values
    speed_mae = float(np.mean(np.abs(pred_speeds - gt_speeds)))
    speed_rmse = float(np.sqrt(np.mean((pred_speeds - gt_speeds)**2)))

    print("[4/4] Generating comparison plots and saving metrics...")
    out_dir = os.path.join(out_root, driver, session)
    os.makedirs(out_dir, exist_ok=True)
    plot_path = os.path.join(out_dir, f"{session}_ai_speed_filter_comparison.png")

    plt.figure(figsize=(12, 5))

    # Subplot 1: Trajectory
    plt.subplot(1, 2, 1)
    plt.plot(e_gt, n_gt, 'k-', linewidth=2.5, label="Ground Truth")
    plt.plot(ai_pos[:, 0], ai_pos[:, 1], 'g-', linewidth=2, label=f"Phase 3 AI Filter ({ai_drift_pct:.2f}%)")
    plt.plot(strapdown_pos[:, 0], strapdown_pos[:, 1], 'r--', linewidth=1.2, alpha=0.7, label=f"Phase 2 Strapdown ({strapdown_drift_pct:.2f}%)")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title(f"Position Comparison ({window_sec:.0f}s Window)\nDrift: {strapdown_drift_pct:.1f}% -> {ai_drift_pct:.1f}%")
    plt.legend()
    plt.axis("equal")
    plt.grid(True, alpha=0.3)

    # Subplot 2: Speed profile
    plt.subplot(1, 2, 2)
    t = np.arange(N) * dt
    plt.plot(t, gt_speeds, 'k-', linewidth=2, label="GT Speed")
    plt.plot(t, pred_speeds, 'b-', linewidth=1.5, alpha=0.85, label=f"AI Pred Speed (MAE={speed_mae:.2f} m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.title(f"Velocity Estimation\nMAE = {speed_mae:.2f} m/s, RMSE = {speed_rmse:.2f} m/s")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()

    print("\n" + "="*60)
    print(f"PHASE 3 EVALUATION RESULTS ({driver} / {session} - {window_sec:.0f}s Window)")
    print("="*60)
    print(f"Ground Truth Distance:           {gt_dist:.2f} m")
    print(f"Phase 2 Strapdown Final Error:   {strapdown_err:.2f} m")
    print(f"Phase 2 Strapdown Drift:         {strapdown_drift_pct:.2f}%")
    print(f"Phase 3 AI Filter Final Error:   {ai_err:.2f} m")
    print(f"Phase 3 AI Filter Drift:         {ai_drift_pct:.2f}%")
    print(f"Drift Reduction:                 {strapdown_drift_pct - ai_drift_pct:.2f}% absolute improvement ({((strapdown_drift_pct - ai_drift_pct)/strapdown_drift_pct)*100:.2f}% reduction)")
    print(f"Speed Filter MAE:                {speed_mae:.2f} m/s")
    print(f"Speed Filter RMSE:               {speed_rmse:.2f} m/s")
    print(f"Comparison plot saved to:        {plot_path}")
    print("="*60 + "\n")

    return {
        "gt_dist": gt_dist,
        "strapdown_drift_pct": strapdown_drift_pct,
        "ai_drift_pct": ai_drift_pct,
        "speed_mae": speed_mae,
        "speed_rmse": speed_rmse,
        "plot_path": plot_path
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate AI Speed Filter vs Phase 2 Baseline")
    parser.add_argument("--driver", default="S (Driver A)")
    parser.add_argument("--session", default="S1")
    parser.add_argument("--model", default="training/models/speed_filter.tflite")
    parser.add_argument("--window", type=float, default=60.0)
    args = parser.parse_args()

    run(driver=args.driver, session=args.session, model_path=args.model, window_sec=args.window)


if __name__ == "__main__":
    main()
