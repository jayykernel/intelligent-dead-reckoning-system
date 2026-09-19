import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.fusion.ai_corrector import AICorrectionModule

def generate():
    # Disable TFLite model for testing
    ai_corrector = AICorrectionModule(model_path="non_existent.tflite", enable_tflite=False, base_sigma_ai=1.0, window_size=5)

    # Create a sequence of IMU samples with varying vibration
    # We'll use 6-dimensional samples: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
    imu_samples = []
    # Low vibration
    for i in range(10):
        imu_samples.append(np.array([0.0, 0.0, 9.81, 0.0, 0.0, 0.0]))  # only gravity
    # Medium vibration
    for i in range(10):
        imu_samples.append(np.array([0.5, 0.5, 9.81, 0.1, 0.1, 0.1]))
    # High vibration
    for i in range(10):
        imu_samples.append(np.array([2.0, 2.0, 9.81, 0.5, 0.5, 0.5]))

    out_states = []

    for sample in imu_samples:
        acc_raw = sample[:3]
        gyro_raw = sample[3:]
        ai_speed, sigma_ai, q_scale = ai_corrector.process_imu_sample(acc_raw, gyro_raw)

        out_states.append({
            "acc": acc_raw.tolist(),
            "gyro": gyro_raw.tolist(),
            "ai_speed": None if ai_speed is None else float(ai_speed),
            "sigma_ai": float(sigma_ai),
            "q_scale": float(q_scale)
        })

    os.makedirs("engine/fusion/tests/vectors", exist_ok=True)
    with open("engine/fusion/tests/vectors/ai_corrector_in.json", "w") as f:
        # We don't need to save the input separately because it's in the output? Actually we need input for the test.
        # Let's save the input sequence and the output sequence.
        pass
    # Instead, let's save the input sequence and the output sequence separately.
    # We'll save the input sequence as a list of imu_samples (each as [acc, gyro])
    input_seq = []
    for sample in imu_samples:
        input_seq.append({
            "acc": sample[:3].tolist(),
            "gyro": sample[3:].tolist()
        })
    with open("engine/fusion/tests/vectors/ai_corrector_in.json", "w") as f:
        json.dump(input_seq, f)
    with open("engine/fusion/tests/vectors/ai_corrector_out.json", "w") as f:
        json.dump(out_states, f)

if __name__ == "__main__":
    generate()
