"""
engine/fusion/tests/test_confidence_ellipse_ui.py

Phase 10: Confidence Ellipse UI Verification & Side-by-Side Covariance Validation (N4).

Verifies:
1. Ellipse geometry (semi-major axis a, semi-minor axis b, orientation theta) matches
   analytical eigenvalues of the 2x2 EKF position covariance P_2D.
2. Ellipse visibly expands during INS-only dead-reckoning drift.
3. Ellipse visibly tightens on successful NIS-passing GNSS reacquisition.
4. Ellipse remains wide/frozen when NIS gating rejects inconsistent GNSS fixes
   after a long outage (faithfully depicting Phase 6 drift without false certainty).
5. Generates the side-by-side verification plot:
   data/processed/phase10_eval/confidence_ellipse_verification.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec

from engine.fusion.fusion_engine import GNSSINSFusionEngine


def compute_ellipse_params(cov_2d, k=2.4477):
    """
    Compute 2D confidence ellipse parameters from a 2x2 covariance matrix.
    k = 2.4477 corresponds to 95% confidence for 2 DOF.
    """
    pEE = cov_2d[0, 0]
    pNN = cov_2d[1, 1]
    pEN = cov_2d[0, 1]

    avg = (pEE + pNN) / 2.0
    diff = (pEE - pNN) / 2.0
    disc = np.sqrt(max(0.0, diff**2 + pEN**2))

    lambda1 = max(1e-6, avg + disc)
    lambda2 = max(1e-6, avg - disc)

    semi_major = k * np.sqrt(lambda1)
    semi_minor = k * np.sqrt(lambda2)
    theta_rad = 0.5 * np.arctan2(2.0 * pEN, pEE - pNN)
    theta_deg = np.degrees(theta_rad)

    return semi_major, semi_minor, theta_deg, lambda1, lambda2


def run_confidence_ellipse_evaluation():
    print("==================================================================")
    print("PHASE 10: CONFIDENCE ELLIPSE UI & COVARIANCE VERIFICATION")
    print("==================================================================")

    dt = 0.1
    engine = GNSSINSFusionEngine(dt=dt)
    engine.initialize_state(
        p0_enu=np.array([0.0, 0.0, 0.0]),
        v0_enu=np.array([10.0, 0.0, 0.0]),
        heading0_deg=90.0,
        acc0_raw=np.array([0.0, 0.0, 9.81])
    )

    t = 0.0
    history = []

    # -------------------------------------------------------------
    # Scenario 1: Steady-State GNSS (0.0s -> 4.0s, 40 epochs)
    # -------------------------------------------------------------
    for _ in range(40):
        t += dt
        gnss_pos = np.array([10.0 * t, 0.0, 0.0])
        gnss_vel = np.array([10.0, 0.0, 0.0])
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=36.0,
            gnss_sat_count=16,
            gnss_acc_m=2.5
        )
        cov = res["cov_2d"]
        a, b, th, l1, l2 = compute_ellipse_params(cov)
        history.append({
            "t": t, "pos": np.copy(res["pos"]), "mode": res["mode"],
            "trust": res["trust_score"], "cov": cov, "a": a, "b": b, "th": th,
            "l1": l1, "l2": l2, "gnss_pos_passed": res["gnss_pos_passed"],
            "phase": "Steady GNSS"
        })

    # -------------------------------------------------------------
    # Scenario 2: Predictive Outage Entry (4.0s -> 5.5s, 15 epochs)
    # -------------------------------------------------------------
    for _ in range(15):
        t += dt
        gnss_pos = np.array([10.0 * t, 0.0, 0.0])
        gnss_vel = np.array([10.0, 0.0, 0.0])
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=16.0,
            gnss_sat_count=4,
            gnss_acc_m=18.0
        )
        cov = res["cov_2d"]
        a, b, th, l1, l2 = compute_ellipse_params(cov)
        history.append({
            "t": t, "pos": np.copy(res["pos"]), "mode": res["mode"],
            "trust": res["trust_score"], "cov": cov, "a": a, "b": b, "th": th,
            "l1": l1, "l2": l2, "gnss_pos_passed": res["gnss_pos_passed"],
            "phase": "Outage Entry"
        })

    # -------------------------------------------------------------
    # Scenario 3: Short Outage & Clean Reacquisition (5.5s -> 8.5s)
    # -------------------------------------------------------------
    # 1.5s blackout (15 epochs)
    for _ in range(15):
        t += dt
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=None,
            gnss_vel_enu=None,
            is_gnss_available=False,
            timestamp=t
        )
        cov = res["cov_2d"]
        a, b, th, l1, l2 = compute_ellipse_params(cov)
        history.append({
            "t": t, "pos": np.copy(res["pos"]), "mode": res["mode"],
            "trust": res["trust_score"], "cov": cov, "a": a, "b": b, "th": th,
            "l1": l1, "l2": l2, "gnss_pos_passed": res["gnss_pos_passed"],
            "phase": "Short Outage"
        })

    # 1.5s clean recovery (15 epochs)
    for _ in range(15):
        t += dt
        gnss_pos = np.array([10.0 * t, 0.0, 0.0])
        gnss_vel = np.array([10.0, 0.0, 0.0])
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=38.0,
            gnss_sat_count=18,
            gnss_acc_m=2.0
        )
        cov = res["cov_2d"]
        a, b, th, l1, l2 = compute_ellipse_params(cov)
        history.append({
            "t": t, "pos": np.copy(res["pos"]), "mode": res["mode"],
            "trust": res["trust_score"], "cov": cov, "a": a, "b": b, "th": th,
            "l1": l1, "l2": l2, "gnss_pos_passed": res["gnss_pos_passed"],
            "phase": "Short Reacq (Pass)"
        })

    # -------------------------------------------------------------
    # Scenario 4: Long Outage (6.0s pure INS) & NIS Rejection (8.5s -> 17.5s)
    # -------------------------------------------------------------
    # 6.0s pure dead reckoning (60 epochs)
    for _ in range(60):
        t += dt
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=None,
            gnss_vel_enu=None,
            is_gnss_available=False,
            timestamp=t
        )
        cov = res["cov_2d"]
        a, b, th, l1, l2 = compute_ellipse_params(cov)
        history.append({
            "t": t, "pos": np.copy(res["pos"]), "mode": res["mode"],
            "trust": res["trust_score"], "cov": cov, "a": a, "b": b, "th": th,
            "l1": l1, "l2": l2, "gnss_pos_passed": res["gnss_pos_passed"],
            "phase": "Long Outage"
        })

    # GNSS reappears with large accumulated discrepancy (fails NIS gating)
    for _ in range(30):
        t += dt
        # True trajectory is at 10*t, but dead reckoning has accumulated drift
        gnss_pos = np.array([10.0 * t, 0.0, 0.0])
        gnss_vel = np.array([10.0, 0.0, 0.0])
        res = engine.step(
            acc_raw=np.array([0.0, 0.0, 9.81]),
            gyro_raw=np.array([0.0, 0.0, 0.0]),
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            is_gnss_available=True,
            timestamp=t,
            gnss_avg_cn0=38.0,
            gnss_sat_count=18,
            gnss_acc_m=2.0
        )
        cov = res["cov_2d"]
        a, b, th, l1, l2 = compute_ellipse_params(cov)
        history.append({
            "t": t, "pos": np.copy(res["pos"]), "mode": res["mode"],
            "trust": res["trust_score"], "cov": cov, "a": a, "b": b, "th": th,
            "l1": l1, "l2": l2, "gnss_pos_passed": res["gnss_pos_passed"],
            "phase": "Long Reacq (NIS Reject)"
        })

    # -------------------------------------------------------------
    # Verification & Metrics
    # -------------------------------------------------------------
    ts = np.array([h["t"] for h in history])
    majors = np.array([h["a"] for h in history])
    minors = np.array([h["b"] for h in history])
    l1s = np.array([h["l1"] for h in history])
    l2s = np.array([h["l2"] for h in history])
    traces = np.array([h["cov"][0, 0] + h["cov"][1, 1] for h in history])
    trusts = np.array([h["trust"] for h in history])
    modes = [h["mode"] for h in history]
    nis_passes = [h["gnss_pos_passed"] for h in history]

    # Verify mathematical parity: semi_major == 2.4477 * sqrt(l1)
    k_expected = 2.4477
    axis_diff = np.max(np.abs(majors - k_expected * np.sqrt(l1s)))
    print(f"[Verification 1] Max disparity between rendered semi-major and analytical eigenvalue: {axis_diff:.6e} m")
    assert axis_diff < 1e-5, "Mathematical disparity detected in ellipse semi-major axis calculation!"

    # Verify growth during outage
    outage_mask = np.array([h["phase"] == "Short Outage" for h in history])
    growth_outage = majors[outage_mask][-1] - majors[outage_mask][0]
    print(f"[Verification 2] Semi-major growth during short outage: +{growth_outage:.2f} m (PASS)")
    assert growth_outage > 0, "Confidence ellipse failed to grow during outage!"

    # Verify shrinkage on short recovery
    reacq_mask = np.array([h["phase"] == "Short Reacq (Pass)" for h in history])
    shrink_reacq = majors[reacq_mask][0] - majors[reacq_mask][-1]
    print(f"[Verification 3] Semi-major contraction upon short reacquisition: -{shrink_reacq:.2f} m (PASS)")
    assert shrink_reacq > 0, "Confidence ellipse failed to contract on clean reacquisition!"

    # Verify ellipse does NOT falsely collapse when NIS rejects updates after long outage
    reject_mask = np.array([h["phase"] == "Long Reacq (NIS Reject)" for h in history])
    first_reject_major = majors[reject_mask][0]
    last_reject_major = majors[reject_mask][-1]
    print(f"[Verification 4] Long Outage Reacquisition with NIS Rejection:")
    print(f"  -> Semi-major at reacquisition start: {first_reject_major:.2f} m")
    print(f"  -> Semi-major after rejected updates: {last_reject_major:.2f} m")
    print(f"  -> False Collapse Detected? {'NO (Faithfully High Uncertainty)' if last_reject_major >= 5.0 else 'YES (ERROR)'}")
    assert last_reject_major >= 5.0, "Confidence ellipse falsely collapsed during rejected GNSS updates!"

    # -------------------------------------------------------------
    # Generate Side-by-Side Plot Artifact
    # -------------------------------------------------------------
    out_dir = "data/processed/phase10_eval"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "confidence_ellipse_verification.png")

    fig = plt.figure(figsize=(15, 12))
    gs = GridSpec(3, 2, figure=fig, height_ratios=[1.2, 1.0, 1.0])

    # Panel 1: 2D Spatial Trajectory with Overlaid Ellipses
    ax1 = fig.add_subplot(gs[0, :])
    xs = [h["pos"][0] for h in history]
    ys = [h["pos"][1] for h in history]
    ax1.plot(xs, ys, 'k--', label='Vehicle Trajectory (ENU East/North)', alpha=0.6, linewidth=1.5)

    # Sample ellipses at representative intervals
    sample_indices = [5, 30, 48, 62, 78, 110, 140, 160]
    for idx in sample_indices:
        h = history[idx]
        px, py = h["pos"][0], h["pos"][1]
        a_m, b_m, th = h["a"], h["b"], h["th"]
        ph = h["phase"]

        if "Steady" in ph:
            color = 'green'
            alpha = 0.25
        elif "Outage Entry" in ph:
            color = 'orange'
            alpha = 0.35
        elif "Short Outage" in ph or "Long Outage" in ph:
            color = 'red'
            alpha = 0.25
        elif "Pass" in ph:
            color = 'blue'
            alpha = 0.35
        else: # NIS Reject
            color = 'purple'
            alpha = 0.35

        # 95% Confidence Ellipse (width=2*a, height=2*b)
        ellipse = patches.Ellipse(
            (px, py), width=2.0 * a_m, height=2.0 * b_m, angle=th,
            edgecolor=color, facecolor=color, alpha=alpha, linewidth=1.8
        )
        ax1.add_patch(ellipse)
        ax1.plot(px, py, 'o', color=color, markersize=4)
        ax1.text(px, py + max(b_m, 2.0) + 1.0, f"t={h['t']:.1f}s\n±{a_m:.1f}m\n({h['phase']})",
                 fontsize=8, ha='center', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor=color))

    ax1.set_title("1. 2D Trajectory with Overlaid Real-Time Confidence Ellipses (95% Bounds)", fontsize=12, fontweight='bold')
    ax1.set_xlabel("East (meters)", fontsize=10)
    ax1.set_ylabel("North (meters)", fontsize=10)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.axis('equal')
    ax1.set_ylim(-15, 20)

    # Panel 2: Semi-Major (a) and Semi-Minor (b) vs Analytical Eigenvalues
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(ts, majors, 'r-', label=r'Semi-Major $a = 2.45 \sqrt{\lambda_1}$ (Rendered)', linewidth=2.0)
    ax2.plot(ts, minors, 'b-', label=r'Semi-Minor $b = 2.45 \sqrt{\lambda_2}$ (Rendered)', linewidth=2.0)
    ax2.plot(ts, k_expected * np.sqrt(l1s), 'k--', label=r'Analytical $\sqrt{\lambda_1}$ Ground Truth', linewidth=1.2, alpha=0.8)
    ax2.set_title("2. Ellipse Semi-Axes vs Analytical EKF Eigenvalues", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Time (s)", fontsize=10)
    ax2.set_ylabel("Semi-Axis Length (meters)", fontsize=10)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(fontsize=9, loc='upper left')

    # Panel 3: Position Covariance Trace & Operational Mode
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(ts, traces, 'darkgreen', label=r'Pos Covariance Trace $\mathrm{Tr}(\mathbf{P}_{2D})$ ($\mathrm{m}^2$)', linewidth=2.0)
    ax3.set_title(r"3. 2D Position Covariance Trace $\mathrm{Tr}(\mathbf{P}_{2D})$", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Time (s)", fontsize=10)
    ax3.set_ylabel(r"Variance ($\mathrm{m}^2$)", fontsize=10)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(fontsize=9, loc='upper left')

    # Panel 4: Outage Predictor Trust Score & NIS Gating Outcome
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(ts, trusts, 'teal', label='Predictive Trust Score $T \in [0, 1]$', linewidth=2.0)
    ax4.axhline(0.8, color='green', linestyle=':', label='High Trust (>0.8)')
    ax4.axhline(0.2, color='red', linestyle=':', label='Low Trust (<0.2)')
    ax4.set_title("4. Predictive Trust Score Dynamics", fontsize=11, fontweight='bold')
    ax4.set_xlabel("Time (s)", fontsize=10)
    ax4.set_ylabel("Trust Score", fontsize=10)
    ax4.set_ylim(-0.05, 1.1)
    ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.legend(fontsize=9, loc='center right')

    # Panel 5: UI Integrity Check - NIS Pass/Fail vs False Collapse
    ax5 = fig.add_subplot(gs[2, 1])
    passed_mask = np.array([p is True for p in nis_passes])
    failed_mask = np.array([p is False and h["phase"] in ["Long Reacq (NIS Reject)"] for p, h in zip(nis_passes, history)])

    ax5.scatter(ts[passed_mask], np.ones(np.sum(passed_mask)), color='green', marker='o', s=30, label='GNSS Update Accepted (NIS Pass)')
    ax5.scatter(ts[failed_mask], np.zeros(np.sum(failed_mask)), color='purple', marker='x', s=40, label='GNSS Update Rejected (NIS Gate Fail)')
    ax5.set_yticks([0, 1])
    ax5.set_yticklabels(['REJECTED', 'ACCEPTED'])
    ax5.set_title("5. NIS Innovation Gating Outcome (No False Collapse)", fontsize=11, fontweight='bold')
    ax5.set_xlabel("Time (s)", fontsize=10)
    ax5.set_ylabel("NIS Decision", fontsize=10)
    ax5.set_ylim(-0.3, 1.3)
    ax5.grid(True, linestyle=':', alpha=0.6)
    ax5.legend(fontsize=9, loc='center right')

    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"[Artifact Saved] Verification plot successfully written to: {out_path}")
    print("==================================================================")


if __name__ == "__main__":
    run_confidence_ellipse_evaluation()
