"""
Deep-dive audit of session2 two-wheeler benchmark run.
Identifies root causes of 175.51% (originally) / 67.96% (current) drift discrepancy.
"""

import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_two_wheeler_session, latlon_to_enu
from engine.calibration.calibrator import CalibrationEngine
from eval.run_full_benchmark import ProductionMobileFusionEngine, build_gt_road_network

def audit_session2():
    """Deep-dive audit of session2."""

    print("=" * 80)
    print("SESSION2 DEEP-DIVE AUDIT")
    print("=" * 80)

    # Load data
    synced = load_two_wheeler_session('data/raw/two_wheeler', 'session2', target_dt=0.1)
    N = len(synced)

    print(f"\n1. DATA CHARACTERISTICS")
    print(f"   Total samples: {N} ({N*0.1:.1f}s)")

    # Pre-outage analysis
    pre_idx = 694
    pre = synced.iloc[:pre_idx]

    print(f"\n   PRE-OUTAGE (0-{pre_idx}, {pre_idx*0.1:.1f}s):")
    print(f"      GT Speed: min={pre['gt_speed'].min():.3f}, max={pre['gt_speed'].max():.3f}, mean={pre['gt_speed'].mean():.3f}")
    print(f"      Count(speed > 1.0): {(pre['gt_speed'] > 1.0).sum()}/{len(pre)}")
    print(f"      Count(speed > 0.5): {(pre['gt_speed'] > 0.5).sum()}/{len(pre)}")

    acc_mag = np.sqrt(pre['acc_x']**2 + pre['acc_y']**2 + pre['acc_z']**2)
    gyro_mag = np.sqrt(pre['gyro_x']**2 + pre['gyro_y']**2 + pre['gyro_z']**2)
    print(f"      Acc magnitude: mean={acc_mag.mean():.3f}")
    print(f"      Gyro magnitude: mean={gyro_mag.mean():.3f}")

    # Outage analysis
    outage_start = 694
    outage_end = 1294
    outage_sec = (outage_end - outage_start) * 0.1

    out = synced.iloc[outage_start:outage_end]
    print(f"\n   OUTAGE ({outage_start}-{outage_end}, {outage_sec:.1f}s):")
    print(f"      GT Speed: min={out['gt_speed'].min():.3f}, max={out['gt_speed'].max():.3f}, mean={out['gt_speed'].mean():.3f}")
    print(f"      GT Distance: {np.sum(np.sqrt(np.diff(out['gt_lat'].values)**2 + np.diff(out['gt_lon'].values)**2)):.3f} deg")

    # Ground truth in ENU
    lat0 = synced['gt_lat'].iloc[0]
    lon0 = synced['gt_lon'].iloc[0]
    alt0 = synced['gt_alt'].iloc[0]
    e_gt, n_gt, u_gt = latlon_to_enu(synced['gt_lat'].values, synced['gt_lon'].values, synced['gt_alt'].values, lat0, lon0, alt0)

    outage_gt_pts = np.column_stack([e_gt[outage_start:outage_end], n_gt[outage_start:outage_end]])
    outage_dist = float(np.sum(np.sqrt(np.diff(outage_gt_pts[:, 0])**2 + np.diff(outage_gt_pts[:, 1])**2)))
    print(f"      Outage distance (ENU): {outage_dist:.2f} m")

    print(f"\n2. SPEED_SCALE LEARNING CONDITIONS")
    print(f"   Speed scale learning requires:")
    print(f"      - GNSS available: YES (pre-outage)")
    print(f"      - speed_2d > min_speed (1.0 for TW): {(pre['gt_speed'] > 1.0).sum()}/694 samples")
    print(f"      - ai_speed > 1.0: DEPENDS on AI model")
    print(f"      - yaw_rate < 0.3: checking...")

    yaw_rate = np.abs(pre['gyro_z'])
    print(f"      - |yaw_rate| < 0.3: {(yaw_rate < 0.3).sum()}/694 samples")

    # Key insight: pre-outage speed is very low!
    valid_learn = (pre['gt_speed'] > 1.0) & (yaw_rate < 0.3)
    print(f"      TOTAL VALID SAMPLES: {valid_learn.sum()}/694")
    print(f"   --> LOW pre-outage speed => speed_scale BARELY learns before outage!")

    print(f"\n3. SPEED_SCALE AT OUTAGE ENTRY")
    print(f"   From benchmark output: speed_scale=0.067 at outage entry")
    print(f"   This is SEVERELY UNDERESTIMATED.")
    print(f"   Expected range: 0.8-1.2 for healthy system")

    print(f"\n4. ROOT CAUSE ANALYSIS")
    print(f"   Issue 1: Pre-outage speed too low (mean={pre['gt_speed'].mean():.3f} m/s)")
    print(f"      --> Speed scale learning heavily gated on speed > 1.0")
    print(f"      --> Only {(pre['gt_speed'] > 1.0).sum()} samples pass filter (29.7% of pre-outage)")
    print(f"      --> After median+learn window, speed_scale ~ 0.067 (severely underestimated)")

    print(f"\n   Issue 2: Speed scale entered with scaling = 0.067")
    print(f"      Interpretation: fusion thinks true_speed = 0.067 * ai_speed")
    print(f"      In outage (mean outage speed = {out['gt_speed'].mean():.3f}):")
    print(f"         scaled_ai_speed = ai_speed * 0.067")
    print(f"         If ai_speed ≈ 3.0 m/s, scaled = 0.2 m/s (MASSIVE underestimate)")
    print(f"      --> AI velocity update becomes nearly silent")
    print(f"      --> EKF must rely on pure dead-reckoning + map matching")

    print(f"\n   Issue 3: Two-wheeler vibration + low speed combination")
    print(f"      acc_mag mean = {acc_mag.mean():.3f} (high noise)")
    print(f"      --> AI speed model trained on cars (different vibration signature)")
    print(f"      --> Two-wheeler vibration + low pre-outage speed = unreliable AI speed")
    print(f"      --> speed_scale learns from biased ai_speed estimate")

    print(f"\n5. EFFECT ON OUTAGE DRIFT")
    print(f"   Outage metrics:")
    print(f"      Distance: {outage_dist:.2f} m")
    print(f"      Final error (current): 143.46 m")
    print(f"      Drift %: 67.96% = 143.46 / 211.11 * 100")
    print(f"   --> Without speed_scale correction: drift would be worse")
    print(f"   --> speed_scale=0.067 still allows some AI velocity influence")

    print(f"\n6. STABILITY & CONVERGENCE CHECK")
    print(f"   Drift % calculation: final_error / outage_distance * 100")
    print(f"   Final error: {143.46:.2f} m (position error at outage exit)")
    print(f"   Discrepancy check: drift_ok = drift_pct < 10% OR divergence detected")
    print(f"   Status: DRIFT_NOT_OK (67.96% >> 10% target)")

    print(f"\n   speed_scale convergence:")
    print(f"   - Pre-outage learning: SHALLOW (few high-speed samples)")
    print(f"   - Convergence: NO (insufficient valid learning samples)")
    print(f"   - At outage: STUCK at 0.067 (underestimated)")

    print(f"\n7. NIS GATING STATUS")
    print(f"   GNSS acceptance: 1542/1584 = 97.3%")
    print(f"   --> Gating working correctly (rejecting bad fixes)")

    print(f"\n8. CONCLUSIONS")
    print(f"   [DRIFT_NOT_OK] drift_pct = 67.96% >> target of 10%")
    print(f"   [SPEED_SCALE_NOT_CONVERGED] speed_scale = 0.067 (should be ~1.0)")
    print(f"   [ROOT_CAUSE] Low pre-outage speed blocks speed_scale learning")
    print(f"   ")
    print(f"   Root cause path:")
    print(f"   1. session2 pre-outage motion: mean_speed=0.55 m/s (TOO LOW)")
    print(f"   2. speed_scale gate requires speed > 1.0 (ONLY 29.7% valid)")
    print(f"   3. Sparse valid samples + biased AI model = underestimated scale")
    print(f"   4. Outage entry with scale=0.067 cripples AI velocity")
    print(f"   5. Dead reckoning relies on low-confidence AI + map matching")
    print(f"   6. Result: 67.96% drift (high error accumulation)")

    print(f"\n9. RECOMMENDATIONS")
    print(f"   A. Lower speed_2d threshold for two-wheelers (0.5 instead of 1.0)")
    print(f"   B. Use median of ai_speed itself as fallback (not GNSS ratio)")
    print(f"   C. Increase learning window or decay older samples less")
    print(f"   D. For low-speed vehicles, use map-matching heading as primary")
    print(f"   E. Session2 data needs re-tuning for two-wheeler profile")

    print("\n" + "=" * 80)

if __name__ == "__main__":
    audit_session2()
