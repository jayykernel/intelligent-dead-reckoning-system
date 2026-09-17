"""
run_strapdown_baseline.py

Phase‑2 – Classical Strapdown INS baseline.

Loads a raw IO‑VNBD session, runs the pure physics strapdown INS (no NHC, no ZUPT, no AI),
produces:
* a trajectory plot (strapdown vs ground‑truth)
* drift percentage for the entire session (or a configurable window)

The script is deliberately simple – it follows the exact exit‑criteria described in
`docs/PHASE_CHECKLIST.md`.
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless run on CI
import matplotlib.pyplot as plt

# Make engine package importable when run from repo root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from engine.strapdown import StrapdownINS
from training.data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu


def run(driver: str, session: str, raw_root: str = "data/raw", out_root: str = "data/processed", window_sec: float = None):
    # Load raw CSVs
    s_df, v_df = load_iovnbd_session(raw_root, driver, session)
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    # Extract needed columns
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    dt = 0.1

    # Initial state from ground truth (first sample)
    gt_speed = synced["gt_speed"].iloc[0]
    gt_heading = synced["gt_heading"].iloc[0]  # clockwise from north
    heading_rad = np.radians(gt_heading)
    v0 = np.array([gt_speed * np.sin(heading_rad), gt_speed * np.cos(heading_rad), 0.0])
    p0 = np.zeros(3)  # ENU origin at first GT point

    # Initialize INS – use first accelerometer sample for leveling
    ins = StrapdownINS()
    ins.initialize_from_gravity_and_heading(p0, v0, acc[0], gt_heading)

    # Run full integration (or truncated window if requested)
    if window_sec is not None:
        N = int(window_sec / dt)
        N = min(N, len(acc))
        acc = acc[:N]
        gyro = gyro[:N]
        synced = synced.iloc[:N]
    N = len(acc)
    pos_hist, vel_hist, quat_hist = ins.run_trajectory(acc, gyro, dt, p0, v0)

    # Ground‑truth ENU for same timestamps
    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, _ = latlon_to_enu(
        synced["gt_lat"].values,
        synced["gt_lon"].values,
        synced["gt_alt"].values,
        lat0, lon0, alt0,
    )

    # Compute distances & drift %
    gt_dist = float(np.sum(np.sqrt(np.diff(e_gt) ** 2 + np.diff(n_gt) ** 2)))
    final_err = np.sqrt((pos_hist[-1, 0] - e_gt[-1]) ** 2 + (pos_hist[-1, 1] - n_gt[-1]) ** 2)
    drift_pct = (final_err / gt_dist) * 100.0 if gt_dist > 0 else np.nan

    # Plot trajectories
    out_dir = os.path.join(out_root, driver, session)
    os.makedirs(out_dir, exist_ok=True)
    plot_path = os.path.join(out_dir, f"{session}_strapdown_drift.png")
    plt.figure(figsize=(6, 6))
    plt.plot(e_gt, n_gt, label="Ground‑truth", linewidth=2)
    plt.plot(pos_hist[:, 0], pos_hist[:, 1], label="Strapdown INS", linewidth=1, alpha=0.7)
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title(f"Strapdown INS drift – {driver} / {session}\nDrift % = {drift_pct:.2f}%")
    plt.legend()
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()

    # Save a compact .npz with the trajectory for later analysis
    npz_path = os.path.join(out_dir, f"{session}_strapdown.npz")
    np.savez_compressed(
        npz_path,
        time=synced["time"].values,
        pos=pos_hist,
        vel=vel_hist,
        quat=quat_hist,
        gt_e=e_gt,
        gt_n=n_gt,
        drift_pct=drift_pct,
    )

    print("[OK] Strapdown baseline completed.")
    print(f"   Session: {driver}/{session}")
    print(f"   Ground-truth distance: {gt_dist:.2f} m")
    print(f"   Final position error: {final_err:.2f} m")
    print(f"   Drift percentage: {drift_pct:.2f}%")
    print(f"   Plot saved to: {plot_path}")
    print(f"   NPZ saved to: {npz_path}")


def main():
    parser = argparse.ArgumentParser(description="Run Phase‑2 classical strapdown baseline")
    parser.add_argument("--driver", default="S (Driver A)")
    parser.add_argument("--session", default="S1")
    parser.add_argument("--window", type=float, default=None, help="Optional window length in seconds (e.g. 60) – if omitted runs full session")
    args = parser.parse_args()
    run(args.driver, args.session, window_sec=args.window)

if __name__ == "__main__":
    main()
