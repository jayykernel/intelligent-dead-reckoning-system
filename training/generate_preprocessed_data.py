"""
generate_preprocessed_data.py

Phase 1 exit-criteria script.

Loads a chosen IO-VNBD session (default: Driver A, S1), runs the preprocessing
pipeline (sync, resample, unit normalization), writes a clean synced NumPy
archive to data/processed/, and saves a sanity-check trajectory plot.

Usage:
    python training/generate_preprocessed_data.py
    python training/generate_preprocessed_data.py --driver "S (Driver A)" --session S4
"""

import os
import sys
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt

# Allow running from repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from data_loader import load_iovnbd_session, preprocess_session, latlon_to_enu


def run(driver: str, session: str, raw_root: str, out_root: str) -> None:
    print(f"[1/4] Loading raw data: {driver} / {session}")
    s_df, v_df = load_iovnbd_session(raw_root, driver, session)
    print(f"      S rows: {len(s_df)}, V rows: {len(v_df)}")

    print("[2/4] Preprocessing (sync + resample at 10 Hz)...")
    synced = preprocess_session(s_df, v_df, target_dt=0.1)
    print(f"      Synced rows: {len(synced)}, duration: {synced['time'].iloc[-1]:.1f} s")

    # Convert both ground-truth and phone GPS to local ENU for plotting
    ref_lat = synced["gt_lat"].iloc[0]
    ref_lon = synced["gt_lon"].iloc[0]
    ref_alt = synced["gt_alt"].iloc[0]

    e_gt, n_gt, _ = latlon_to_enu(
        synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values,
        ref_lat, ref_lon, ref_alt,
    )
    e_ph, n_ph, _ = latlon_to_enu(
        synced["phone_lat"].values, synced["phone_lon"].values, synced["phone_alt"].values,
        ref_lat, ref_lon, ref_alt,
    )

    # Compute quick stats for the operator
    gt_path_len = float(np.sum(np.sqrt(np.diff(e_gt) ** 2 + np.diff(n_gt) ** 2)))
    print(f"      GT path length: {gt_path_len:.1f} m")
    print(f"      Accel range (acc_x): [{synced['acc_x'].min():.3f}, {synced['acc_x'].max():.3f}] m/s²")
    print(f"      Gyro range (gyro_z): [{synced['gyro_z'].min():.4f}, {synced['gyro_z'].max():.4f}] rad/s")

    print("[3/4] Saving synced NPZ...")
    # Build output directory: out_root / driver / session/
    out_dir = os.path.join(out_root, driver, session)
    os.makedirs(out_dir, exist_ok=True)

    npz_path = os.path.join(out_dir, f"{session}_synced.npz")
    np.savez_compressed(
        npz_path,
        time=synced["time"].values,
        acc_x=synced["acc_x"].values,
        acc_y=synced["acc_y"].values,
        acc_z=synced["acc_z"].values,
        gyro_x=synced["gyro_x"].values,
        gyro_y=synced["gyro_y"].values,
        gyro_z=synced["gyro_z"].values,
        mag_x=synced["mag_x"].values,
        mag_y=synced["mag_y"].values,
        mag_z=synced["mag_z"].values,
        phone_lat=synced["phone_lat"].values,
        phone_lon=synced["phone_lon"].values,
        phone_alt=synced["phone_alt"].values,
        phone_speed=synced["phone_speed"].values,
        phone_heading=synced["phone_heading"].values,
        gt_lat=synced["gt_lat"].values,
        gt_lon=synced["gt_lon"].values,
        gt_alt=synced["gt_alt"].values,
        gt_speed=synced["gt_speed"].values,
        gt_heading=synced["gt_heading"].values,
    )
    print(f"      Saved: {npz_path}")

    print("[4/4] Saving sanity-check trajectory plot...")
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Left: ENU trajectory
    ax0 = axes[0]
    ax0.plot(e_gt, n_gt, label="Ground-truth (vehicle)", linewidth=2, color="tab:blue")
    ax0.plot(e_ph, n_ph, label="Phone GPS", linewidth=1, alpha=0.7, color="tab:orange")
    ax0.set_xlabel("East (m)")
    ax0.set_ylabel("North (m)")
    ax0.set_title(f"Trajectory — {driver} / {session}")
    ax0.legend(fontsize=8)
    ax0.axis("equal")
    ax0.grid(True, alpha=0.3)

    # Right: Speed comparison
    ax1 = axes[1]
    ax1.plot(synced["time"], synced["gt_speed"], label="Ground-truth speed (m/s)", linewidth=1.5, color="tab:blue")
    ax1.plot(synced["time"], synced["phone_speed"], label="Phone GPS speed (m/s)", linewidth=1, alpha=0.7, color="tab:orange")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Speed (m/s)")
    ax1.set_title("Speed comparison")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(out_dir, f"{session}_traj.png")
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"      Saved: {plot_path}")

    print("\n[OK] Phase 1 exit-criteria script completed successfully.")
    print(f"   NPZ:  {npz_path}")
    print(f"   Plot: {plot_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Phase 1 preprocessing pipeline demo")
    parser.add_argument("--driver", default="S (Driver A)",
                        help="Driver folder name (e.g. 'S (Driver A)')")
    parser.add_argument("--session", default="S1",
                        help="Session identifier (e.g. S1)")
    parser.add_argument("--raw-root", default="data/raw",
                        help="Path to data/raw directory")
    parser.add_argument("--out-root", default="data/processed",
                        help="Path to data/processed directory")
    args = parser.parse_args()
    run(args.driver, args.session, args.raw_root, args.out_root)


if __name__ == "__main__":
    main()
