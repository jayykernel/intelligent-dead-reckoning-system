import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.calibration.calibrator import CalibrationEngine
from training.data_loader import load_iovnbd_session, preprocess_session

def generate_calibrator_test_vectors():
    print("Generating calibrator test vectors...")
    calib = CalibrationEngine()

    # Load a small snippet from a real run
    s_df, v_df = load_iovnbd_session("data/raw", "S (Driver A)", "S4")
    synced = preprocess_session(s_df, v_df, target_dt=0.1)

    acc = synced[["acc_x", "acc_y", "acc_z"]].values[:1200]
    gyro = synced[["gyro_x", "gyro_y", "gyro_z"]].values[:1200]
    speed = synced["gt_speed"].values[:1200]
    speed = np.nan_to_num(speed, nan=0.0)

    success = calib.calibrate_from_session(acc, gyro, speed, dt=0.1)

    # We will export:
    # 1. Inputs (first 50 samples of acc, gyro, speed for an incremental test maybe?)
    # Calibrator currently works as batch `calibrate_from_session`.
    # The Kotlin port needs to match `calibrate_from_session`.

    # Export the batch inputs
    input_data = {
        "acc": acc.tolist(),
        "gyro": gyro.tolist(),
        "speed": speed.tolist(),
        "dt": 0.1
    }

    output_data = {
        "success": bool(success),
        "R_phone_to_veh": calib.R_phone_to_veh.tolist(),
        "gyro_bias": calib.gyro_bias.tolist(),
        "accel_bias": calib.accel_bias.tolist(),
        "alignment_score": float(calib.alignment_score)
    }

    os.makedirs("engine/calibration/tests/vectors", exist_ok=True)
    with open("engine/calibration/tests/vectors/calibrator_in.json", "w") as f:
        json.dump(input_data, f)
    with open("engine/calibration/tests/vectors/calibrator_out.json", "w") as f:
        json.dump(output_data, f)
    print("Saved calibrator_in.json and calibrator_out.json")

if __name__ == "__main__":
    generate_calibrator_test_vectors()
