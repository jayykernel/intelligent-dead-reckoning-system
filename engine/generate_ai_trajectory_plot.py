"""
engine/generate_ai_trajectory_plot.py

Generates standalone AI-corrected trajectory plots (AI estimated path vs
ground truth) for the Screening Package.  Uses the same dead-reckoning
logic as run_ai_speed_filter_baseline.py.
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


def generate(
    driver: str,
    session: str,
    model_path: str = "training/models/speed_filter.tflite",
    raw_root: str = "data/raw",
    out_root: str = "screening_package/plots",
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

    print("[2/4] Running strapdown INS (for gyro-derived attitude)...")
    ins = StrapdownINS()
    ins.initialize_from_gravity_and_heading(p0, v0, acc[0], gt_heading_0)
    strapdown_pos, strapdown_vel, strapdown_quat = ins.run_trajectory(acc, gyro, dt, p0, v0)

    print(f"[3/4] Running AI Speed Filter ({model_path})...")
    speed_filter = SpeedFilter(model_path)
    pred_speeds = speed_filter.predict_sequence(imu_6d)

    # Dead reckoning: AI speed + gyro-integrated heading
    ai_pos = np.zeros((N, 3))
    ai_vel = np.zeros((N, 3))
    ai_pos[0] = p0
    ai_vel[0] = v0

    for i in range(1, N):
        qw, qx, qy, qz = strapdown_quat[i]
        psi = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
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

    # Metrics
    gt_dist = float(np.sum(np.sqrt(np.diff(e_gt) ** 2 + np.diff(n_gt) ** 2)))
    ai_err = np.sqrt((ai_pos[-1, 0] - e_gt[-1])**2 + (ai_pos[-1, 1] - n_gt[-1])**2)
    ai_drift_pct = (ai_err / gt_dist) * 100.0 if gt_dist > 0 else np.nan

    print("[4/4] Generating plot...")
    os.makedirs(out_root, exist_ok=True)
    plot_path = os.path.join(out_root, f"{session}_ai_corrected_trajectory.png")

    plt.figure(figsize=(8, 8))
    plt.plot(e_gt, n_gt, "k-", linewidth=2.5, label="Ground Truth")
    plt.plot(ai_pos[:, 0], ai_pos[:, 1], "g-", linewidth=2,
             label=f"Phase 3 AI Filter (drift {ai_drift_pct:.1f}%)")
    plt.plot(e_gt[0], n_gt[0], "ko", markersize=8, label="Start")
    plt.plot(e_gt[-1], n_gt[-1], "k^", markersize=8, label="End (GT)")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title(f"{session} — AI-Corrected Trajectory vs Ground Truth\n"
              f"({window_sec:.0f}s window, drift {ai_drift_pct:.1f}%, "
              f"error {ai_err:.1f} m / {gt_dist:.1f} m)")
    plt.legend()
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()

    print(f"Saved: {plot_path}")
    return plot_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate AI-corrected trajectory plot for Screening Package")
    parser.add_argument("--driver", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--model", default="training/models/speed_filter.tflite")
    parser.add_argument("--window", type=float, default=60.0)
    args = parser.parse_args()

    generate(driver=args.driver, session=args.session,
             model_path=args.model, window_sec=args.window)


if __name__ == "__main__":
    main()
