"""
Phase 6 Evaluation: GNSS+INS Fusion
- EKF-based fusion
- Chi-squared NIS gating
- AI correction module (MEMS path)
- Magnetometer gating

Simulates GNSS outages and evaluates performance against ground truth (N4 Confidence Ellipses plotted).
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from engine.fusion.fusion_engine import GNSSINSFusionEngine
from training.data_loader import load_iovnbd_session, preprocess_session, load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine

def plot_covariance_ellipse(ax, pos, cov, n_std=3.0, **kwargs):
    """Plot covariance ellipse for 2D position."""
    # Eigenvalues and eigenvectors
    vals, vecs = np.linalg.eigh(cov)
    # Ensure eigenvalues are positive
    vals = np.maximum(vals, 1e-6)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
    width, height = 2 * n_std * np.sqrt(vals)
    ellipse = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    ax.add_patch(ellipse)

def run_evaluation(
    data_type: str,
    driver: str,
    session: str,
    raw_root: str = "data/raw",
    outage_mode: str = "middle_60s"
):
    print(f"[{data_type.upper()}] Loading session {driver} / {session}...")
    if data_type == "car":
        s_df, v_df = load_iovnbd_session(raw_root, driver, session)
        synced = preprocess_session(s_df, v_df, target_dt=0.1)
    else:
        synced = load_two_wheeler_session(os.path.join(raw_root, "two_wheeler"), session)

    N = len(synced)
    dt = 0.1
    print(f"  Using {N} samples ({N*dt:.1f} seconds)")

    # 1. Calibration
    calib = CalibrationEngine()
    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = synced["gt_speed"].values
    speed = np.nan_to_num(speed, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    calib_success = calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
    if not calib_success:
        print("  WARNING: Calibration failed, using identity transformation and zero bias.")

    # 2. Init Fusion Engine
    fusion = GNSSINSFusionEngine(dt=dt)

    lat0 = synced["gt_lat"].iloc[0]
    lon0 = synced["gt_lon"].iloc[0]
    alt0 = synced["gt_alt"].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced["gt_lat"].values, synced["gt_lon"].values, synced["gt_alt"].values, lat0, lon0, alt0)

    gt_heading = synced["gt_heading"].values
    if np.any(np.isnan(gt_heading)):
        gt_heading = np.interp(np.arange(len(gt_heading)), np.where(~np.isnan(gt_heading))[0], gt_heading[~np.isnan(gt_heading)])

    p0 = np.array([e_gt[0], n_gt[0], u_gt[0]])
    heading_rad = np.radians(gt_heading[0])
    v0 = np.array([speed[0] * np.sin(heading_rad), speed[0] * np.cos(heading_rad), 0.0])

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    # Calculate outage indices (60 seconds in the middle)
    outage_start = int(N * 0.4)
    outage_end = outage_start + int(60.0 / dt)
    outage_end = min(outage_end, N - int(10.0 / dt))

    results = []
    outage_path = []

    print("  Running Fusion Pipeline...")
    for i in range(1, N):
        in_outage = (outage_start <= i <= outage_end)

        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None

        # We also pass GNSS velocity to maintain heading observability during GNSS-aided phase
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None

        m_raw = mag[i] if mag is not None else None

        res = fusion.step(
            acc_raw=acc[i],
            gyro_raw=gyro[i],
            mag_raw=m_raw,
            gnss_pos_enu=pos_enu,
            gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage,
            timestamp=i*dt
        )
        results.append(res)

        if in_outage:
            outage_path.append([e_gt[i], n_gt[i]])

    # Metrics & Evaluation
    pos_est = np.array([r["pos"] for r in results])
    cov_est = np.array([r["cov_2d"] for r in results])

    pos_est = np.vstack([p0, pos_est])  # Align length N
    yaw_est = np.array([fusion.trajectory_euler[i][2] for i in range(N)])

    # Outage Drift Metrics
    outage_path = np.array(outage_path)
    if len(outage_path) > 0:
        outage_dist = float(np.sum(np.sqrt(np.diff(outage_path[:, 0])**2 + np.diff(outage_path[:, 1])**2)))
        final_err = float(np.linalg.norm(pos_est[outage_end, :2] - np.array([e_gt[outage_end], n_gt[outage_end]])))
        drift_pct = (final_err / outage_dist) * 100.0 if outage_dist > 0 else 0.0
    else:
        outage_dist = 0.0
        final_err = 0.0
        drift_pct = 0.0

    # Heading Errors during outage
    yaw_est_outage = yaw_est[outage_start:outage_end]
    yaw_gt_outage = gt_heading[outage_start:outage_end]

    heading_diffs = np.abs((yaw_est_outage - yaw_gt_outage + 180) % 360 - 180)
    heading_error_mean = float(np.mean(heading_diffs)) if len(heading_diffs) > 0 else 0.0
    heading_error_rms = float(np.sqrt(np.mean(heading_diffs**2))) if len(heading_diffs) > 0 else 0.0
    heading_error_final = float(heading_diffs[-1]) if len(heading_diffs) > 0 else 0.0

    # NIS Logging Statistics
    nis_history = fusion.ekf.nis_history
    gnss_updates = [n for n in nis_history if n["type"] == "GNSS_POS"]
    passed_gnss = sum(1 for n in gnss_updates if n["passed"])
    rejected_gnss = len(gnss_updates) - passed_gnss

    mag_updates = [n for n in nis_history if n["type"] == "MAG_HEADING"]
    passed_mag = sum(1 for n in mag_updates if n["passed"])
    rejected_mag = len(mag_updates) - passed_mag

    print(f"[{data_type.upper()}] Results for 60s GNSS Outage:")
    print(f"  Outage Distance:                 {outage_dist:.2f} m")
    print(f"  Final Position Error (Phase 6):  {final_err:.2f} m")
    print(f"  Phase 6 Drift %:                 {drift_pct:.2f}%")
    print(f"  Heading Error (Mean):            {heading_error_mean:.2f} deg")
    print(f"  Heading Error (RMS):             {heading_error_rms:.2f} deg")
    print(f"  Heading Error (Final):           {heading_error_final:.2f} deg")
    print(f"  NIS GNSS (Passed/Rejected):      {passed_gnss} / {rejected_gnss}")
    print(f"  Mag Gates (Passed/Rejected):     {passed_mag} / {rejected_mag}")

    # Generate Plot
    out_dir = os.path.join("data/processed/phase6_eval", data_type, session)
    os.makedirs(out_dir, exist_ok=True)
    plot_path = os.path.join(out_dir, f"{session}_phase6_ellipse_comparison.png")

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(e_gt, n_gt, 'k-', linewidth=2, label="Ground Truth", alpha=0.6)

    # Plot Trajectory colored by Mode
    ax.plot(pos_est[:outage_start, 0], pos_est[:outage_start, 1], 'b-', linewidth=2, label="GNSS-Aided")
    ax.plot(pos_est[outage_start:outage_end, 0], pos_est[outage_start:outage_end, 1], 'r-', linewidth=2, label="Pure Dead Reckoning (Blackout)")
    ax.plot(pos_est[outage_end:, 0], pos_est[outage_end:, 1], 'b-', linewidth=2)

    # Plot Covariance Ellipses every 5 seconds
    step_samples = int(5.0 / dt)
    for idx in range(0, N, step_samples):
        color = 'red' if (outage_start <= idx <= outage_end) else 'blue'
        alpha = 0.3 if (outage_start <= idx <= outage_end) else 0.1
        plot_covariance_ellipse(ax, pos_est[idx, :2], cov_est[idx], n_std=3.0, edgecolor=color, facecolor=color, alpha=alpha)

    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title(f"Phase 6 GNSS+INS Fusion - 60s Outage ({session})\nDrift: {drift_pct:.2f}% | Max Ellipse Error: {final_err:.2f}m")
    ax.legend(loc='lower left')

    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()

    print(f"  Plot saved to: {plot_path}")
    print("-" * 50)

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Evaluate Phase 6 GNSS+INS Fusion Engine")
    parser.add_argument("--data-type", choices=["car", "tw"], required=True)
    parser.add_argument("--driver", default="S (Driver A)")
    parser.add_argument("--session", default="S1")
    parser.add_argument("--raw-root", default="data/raw")
    args = parser.parse_args()

    run_evaluation(
        data_type=args.data_type,
        driver=args.driver,
        session=args.session,
        raw_root=args.raw_root
    )

if __name__ == "__main__":
    main()