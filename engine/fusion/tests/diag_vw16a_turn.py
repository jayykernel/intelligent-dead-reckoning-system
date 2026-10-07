import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from engine.fusion.tests.diag_vw16a_deep import step_records, outage_recs

for t_idx in range(480, 601, 10):
    r = outage_recs[t_idx]
    err = np.linalg.norm(r['pos_est'][:2] - r['pos_gt'][:2])
    h_err = ((r['heading_est'] - r['heading_gt'] + 180) % 360) - 180
    mm = r['last_mm']
    snapped = mm.snapped if mm is not None else False
    seg_id = mm.matched_segment_id if mm is not None else None
    print(f"t={r['t']:.1f}s: pos_err={err:6.2f}m, head_est={r['heading_est']:6.2f}, head_gt={r['heading_gt']:6.2f}, h_err={h_err:+6.2f}deg, mm_snapped={snapped}, seg_id={seg_id}")
