import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.fusion.tests.diag_vw16a_deep import step_records, outage_recs

# Let's see the trajectory comparison in 2D
# Plot or inspect positions along the outage
pos_est = np.array([r['pos_est'] for r in outage_recs])
pos_gt = np.array([r['pos_gt'] for r in outage_recs])
heading_est = np.array([r['heading_est'] for r in outage_recs])
heading_gt = np.array([r['heading_gt'] for r in outage_recs])
ai_speeds = np.array([r['ai_speed'] for r in outage_recs])
speed_gt = np.array([r['speed_gt'] for r in outage_recs])
speed_scale = outage_recs[0]['speed_scale']

# Cumulative distance
dt = 0.1
int_speed_scaled = np.sum(ai_speeds * speed_scale * dt)
int_speed_unscaled = np.sum(ai_speeds * 1.0 * dt)
int_speed_gt = np.sum(speed_gt * dt)

print(f"Integrated speed (scaled AI speed = {speed_scale:.3f} * AI): {int_speed_scaled:.2f} m")
print(f"Integrated speed (unscaled AI speed): {int_speed_unscaled:.2f} m")
print(f"Integrated speed (GT): {int_speed_gt:.2f} m")

# Look at speed during outage:
print(f"Mean speed during outage: GT={np.mean(speed_gt):.2f} m/s, AI*scale={np.mean(ai_speeds*speed_scale):.2f} m/s, AI={np.mean(ai_speeds):.2f} m/s")

# Let's inspect where the error grows:
for t_idx in [0, 100, 200, 300, 400, 500, 600]:
    if t_idx < len(outage_recs):
        r = outage_recs[t_idx]
        err = np.linalg.norm(r['pos_est'][:2] - r['pos_gt'][:2])
        h_err = ((r['heading_est'] - r['heading_gt'] + 180) % 360) - 180
        print(f"t={r['t']:.1f}s (step {t_idx}): pos_err={err:.2f}m, head_err={h_err:+.2f}deg, speed_est={np.linalg.norm(r['vel_est'][:2]):.2f}m/s, speed_gt={r['speed_gt']:.2f}m/s")
