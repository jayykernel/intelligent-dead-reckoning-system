"""
engine/fusion/tests/debug_ekf_vta28.py
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from eval.run_full_benchmark import (
    ProductionMobileFusionEngine,
    build_gt_road_network,
    evaluate_session_dead_reckoning
)
from training.data_loader import load_iovnbd_session

s_df, v_df = load_iovnbd_session("data/raw", "Vta (Driver E)", "Vta28")
res = evaluate_session_dead_reckoning(
    session_tuple=(s_df, v_df),
    session_name="Vta28",
    vehicle_type="car",
    outage_duration_s=60.0
)
print("Result:", res)
