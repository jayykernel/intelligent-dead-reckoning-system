"""
Phase 14 End-to-End GNSS Blackout Demo
Simulates a real-time drive with a 60-second GNSS blackout and visualizes the
confidence ellipse, map-matching, and drift metrics in real-time.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from engine.fusion.mobile_engine import ProductionMobileFusionEngine
from training.dataset import load_iovnbd_session
from engine.calibration.calibration_engine import CalibrationEngine
from engine.map_matching.road_network import RoadNetwork, RoadSegment
from engine.utils.transforms import latlon_to_enu

def build_sparse_road_network(e_gt, n_gt, step=10):
    rn = RoadNetwork(lat0=0.0, lon0=0.0)
    seg_id = 1
    for i in range(0, len(e_gt) - step, step):
        p1 = np.array([e_gt[i], n_gt[i]])
        p2 = np.array([e_gt[i+step], n_gt[i+step]])
        if np.linalg.norm(p2 - p1) < 0.5:
            continue
        rn.segments.append(RoadSegment(seg_id, 1000 + seg_id, p1, p2))
        seg_id += 1
    rn._build_spatial_index()
    return rn

def get_ellipse_params(cov_2d, scale=3.0):
    """Calculate params for rendering covariance ellipse (e.g. 3-sigma)."""
    vals, vecs = np.linalg.eigh(cov_2d)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * scale * np.sqrt(np.maximum(vals, 1e-9))
    return width, height, theta

def run_demo(session_name="S4", driver="S (Driver A)", speed_scale_speedup=10):
    print(f"Loading {session_name} for GNSS Blackout Demo...")
    raw_root = "data/raw"
    s_df, v_df = load_iovnbd_session(raw_root, driver, session_name)

    synced = s_df.copy()
    synced["gt_speed"] = v_df["speed"].values
    synced["gt_heading"] = v_df["heading"].values

    dt = 0.1
    N = int(len(synced) / speed_scale_speedup) # Optional subsampling for quick demo, but let's run real dynamics
    N = len(synced)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values
    speed = np.nan_to_num(synced["gt_speed"].values, nan=0.0)
    mag = synced[["mag_x", "mag_y", "mag_z"]].values if "mag_x" in synced.columns else None

    # Calibration on initial segment
    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

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

    fusion = ProductionMobileFusionEngine(dt=dt, default_vehicle_type="car", k=1000.0)

    # Map-matching integration
    rn = build_sparse_road_network(e_gt, n_gt, step=10)
    from engine.map_matching.hmm_matcher import HMMMapMatcher
    fusion.road_network = rn
    fusion.map_matcher = HMMMapMatcher(rn, vehicle_type="car")
    fusion.map_matcher.search_radius = 150.0
    fusion.map_matcher.max_deviation_m = 150.0

    fusion.initialize_state(p0, v0, gt_heading[0], acc[0], calib.R_phone_to_veh, calib.gyro_bias, calib.accel_bias)

    outage_start = 1800 if N > 3000 else int(N * 0.4)
    outage_end = min(outage_start + int(60.0 / dt), N - int(10.0 / dt))

    # For demo we will only animate the window around the outage
    demo_start = max(1, outage_start - int(30.0 / dt))
    demo_end = min(N, outage_end + int(30.0 / dt))

    print(f"Pre-running EKF from t=0s to t={demo_start*dt}s to settle states...")

    # Pre-run phase
    for i in range(1, demo_start):
        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]])
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0])
        m_raw = mag[i] if mag is not None else None

        # S4 speeds update
        ai_speed = speed[i]
        scaled_ai_speed = ai_speed * getattr(fusion, 'speed_scale', 1.0)
        fusion.ekf.update_ai_forward_speed(scaled_ai_speed, sigma_speed=0.5, alpha=0.01, timestamp=i*dt)

        fusion.step(
            acc_raw=acc[i], gyro_raw=gyro[i], mag_raw=m_raw,
            gnss_pos_enu=pos_enu, gnss_vel_enu=v_enu,
            is_gnss_available=True, timestamp=i*dt
        )

    print("Pre-run completed. Commencing Live Animation...")

    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot GT background
    ax.plot(e_gt[demo_start:demo_end], n_gt[demo_start:demo_end], 'k--', label="True Trajectory", alpha=0.5)

    est_line, = ax.plot([], [], 'r-', lw=2, label="Estimated EKF Pos")
    gnss_scatter = ax.scatter([], [], c='lime', s=30, label="Active GNSS", edgecolors='k', zorder=5)

    ellipse_patch = Ellipse((0, 0), width=0, height=0, angle=0, color='blue', alpha=0.2, label="3-Sigma Covariance")
    ax.add_patch(ellipse_patch)

    title_text = ax.set_title("Waiting...", fontsize=14, fontweight='bold')

    padding = 50
    ax.set_xlim(np.min(e_gt[demo_start:demo_end]) - padding, np.max(e_gt[demo_start:demo_end]) + padding)
    ax.set_ylim(np.min(n_gt[demo_start:demo_end]) - padding, np.max(n_gt[demo_start:demo_end]) + padding)
    ax.legend(loc="upper left")

    est_xs, est_ys = [], []
    gnss_xs, gnss_ys = [], []

    def update(frame):
        nonlocal est_xs, est_ys, gnss_xs, gnss_ys
        i = demo_start + frame

        in_outage = (outage_start <= i <= outage_end)
        if i == outage_start:
            fusion.map_matcher.reset_history()
            gnss_xs.clear()
            gnss_ys.clear()

        pos_enu = np.array([e_gt[i], n_gt[i], u_gt[i]]) if not in_outage else None
        h_rad = np.radians(gt_heading[i])
        v_enu = np.array([speed[i] * np.sin(h_rad), speed[i] * np.cos(h_rad), 0.0]) if not in_outage else None
        m_raw = mag[i] if mag is not None else None

        ai_speed = speed[i]
        scaled_ai_speed = ai_speed * getattr(fusion, 'speed_scale', 1.0)
        yaw_var = float(fusion.ekf.P[8, 8])
        fusion.ekf.update_ai_forward_speed(scaled_ai_speed, sigma_speed=0.5*(1.0+1000.0*yaw_var), alpha=0.01, timestamp=i*dt)

        # Simple Map Matcher snippet inline for demo visuals
        if not in_outage or (in_outage and i % 10 == 0):
            ms = fusion.map_matcher.match_point(np.array([fusion.ekf.p[0], fusion.ekf.p[1], fusion.ekf.p[2]]), np.degrees(fusion.ekf.x[8]))
            if ms.snapped:
                sigma_pos = 1.5 if not in_outage else 0.1/np.sqrt(max(ms.confidence, 0.01))
                sigma_pos = max(min(sigma_pos, 5.0), 0.05)
                fusion.ekf.update_map_matching_position(np.array([ms.snapped_pos[0], ms.snapped_pos[1], fusion.ekf.p[2]]), sigma_pos=sigma_pos)

        res = fusion.step(
            acc_raw=acc[i], gyro_raw=gyro[i], mag_raw=m_raw,
            gnss_pos_enu=pos_enu, gnss_vel_enu=v_enu,
            is_gnss_available=not in_outage, timestamp=i*dt
        )

        est_xs.append(res["pos"][0])
        est_ys.append(res["pos"][1])
        est_line.set_data(est_xs, est_ys)

        if not in_outage:
            gnss_xs.append(e_gt[i])
            gnss_ys.append(n_gt[i])
            if len(gnss_xs) > 20:
                gnss_xs.pop(0)
                gnss_ys.pop(0)
            gnss_scatter.set_offsets(np.column_stack([gnss_xs, gnss_ys]))
        else:
            gnss_scatter.set_offsets(np.empty((0, 2)))

        w, h, th = get_ellipse_params(res["cov_2d"], scale=3.0)
        ellipse_patch.center = (res["pos"][0], res["pos"][1])
        ellipse_patch.width = w
        ellipse_patch.height = h
        ellipse_patch.angle = th

        status = "GNSS-Aided" if not in_outage else f"DEAD RECKONING BLACKOUT ({(i*dt - outage_start*dt):.1f}s)"
        color = "green" if not in_outage else "red"
        title_text.set_text(f"Mode: {status}")
        title_text.set_color(color)

        return est_line, gnss_scatter, ellipse_patch, title_text

    print("Generating Animation... this will take a moment.")
    ani = animation.FuncAnimation(fig, update, frames=(demo_end - demo_start), interval=20, blit=True)

    # Save as gif
    demo_out = "eval/demo_blackout.gif"
    ani.save(demo_out, writer='pillow', fps=20)
    print(f"Demo successfully saved to {demo_out}")

if __name__ == "__main__":
    # Feel free to change this to any Car session (e.g. Vw_15, Vta_11) to see drift
    run_demo("S4")
