import sys
import os
import numpy as np
import torch

# Ensure we are in the project root
project_root = r"C:\Users\JAYANITHYAN M R\OneDrive\Documents\dead reckoning proto"
os.chdir(project_root)
sys.path.insert(0, project_root)

from core.models.dataset_generator import SyntheticTrajectoryGenerator
from core.models.velocity_estimator import Velocity1DCNN
from core.models.dataset_interfaces import DatasetAdapter, VehicleClass

def main():
    torch.manual_seed(42)
    np.random.seed(42)

    print("Loading trained model...")
    model_path = os.path.join(project_root, 'velocity_model.pth')
    model = Velocity1DCNN()
    model.load_state_dict(torch.load(model_path))
    model.eval()
    print(f"Model loaded from {model_path}")

    # Check on seed 666 (held out seed used in final_test.py)
    print("\nGenerating trajectory with seed=666 (held out)...")
    gen = SyntheticTrajectoryGenerator(seed=666)
    traj = gen.generate_straight_accel_decel(duration=30.0, max_speed=20.0)

    print(f"Trajectory length: {len(traj['time'])} samples at {1/(traj['time'][1]-traj['time'][0]):.1f} Hz")
    print(f"Duration: {traj['time'][-1]:.1f} s")

    seq = DatasetAdapter.from_continuous_arrays(
        trajectory_id="check_traj",
        vehicle_class=VehicleClass.CAR,
        times_s=traj['time'],
        accel_v=traj['accel_v'],
        gyro_v=traj['gyro_v'],
        vel_v=traj['vel_v'],
        window_size=100,
        stride=10
    )
    raw_preds = []
    gts = []
    for sample in seq.samples:
        feat = torch.from_numpy(sample.features.data).unsqueeze(0)
        with torch.no_grad():
            v_pred, _ = model(feat)
            raw_preds.append(v_pred.item())
        gts.append(sample.truth.velocity_forward_mps)
    raw_preds = np.array(raw_preds)
    gts = np.array(gts)
    errors = raw_preds - gts
    mae = np.mean(np.abs(errors))
    rmse = np.sqrt(np.mean(errors**2))
    print(f"ML window-level MAE: {mae:.3f} m/s")
    print(f"ML window-level RMSE: {rmse:.3f} m/s")
    print(f"Number of windows: {len(seq.samples)}")

if __name__ == "__main__":
    main()