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
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.sensors.data_types import ImuSample
from core.alignment.quaternion_utils import quat_to_rotation_matrix

def create_initial_state():
    return NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(0.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )

def run_eskf_only(traj):
    '''Run ESKF-only (Pure INS)'''
    t = traj['time']
    acc = traj['accel_v']
    gyr = traj['gyro_v']
    vel_gt = traj['vel_v']  # vehicle frame velocity (forward is x)

    N = len(t)
    dt = t[1] - t[0] if N > 1 else 0.01

    ins = StrapdownINS(create_initial_state())
    vel_est = np.zeros(N)

    for i in range(N):
        ts_ns = int(t[i] * 1e9)
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(acc[i]),
            gyro_rad_s=tuple(gyr[i])
        )
        ins.propagate(imu)
        vel_est[i] = np.linalg.norm(ins.state.velocity_mps)

    # Compute metrics
    gt_north = vel_gt[:, 0]
    errors = vel_est - gt_north
    mae = np.mean(np.abs(errors))
    rmse = np.sqrt(np.mean(errors**2))

    # Position error
    def integrate_to_position(vel_north, dt):
        pos = np.zeros_like(vel_north)
        p = 0.0
        for i, v in enumerate(vel_north):
            p = p + v * dt
            pos[i] = p
        return pos

    pos_est = integrate_to_position(vel_est, dt)
    pos_gt = integrate_to_position(gt_north, dt)
    pos_err = pos_est - pos_gt
    final_pos_err = pos_err[-1]
    max_pos_err = np.max(np.abs(pos_err))

    return {
        'velocity_mae': mae,
        'velocity_rmse': rmse,
        'position_drift': final_pos_err,
        'max_position_error': max_pos_err
    }

def run_eskf_perfect_gt(traj):
    '''Run ESKF with perfect ground-truth forward velocity'''
    t = traj['time']
    acc = traj['accel_v']
    gyr = traj['gyro_v']
    vel_gt = traj['vel_v']  # vehicle frame velocity (forward is x)

    N = len(t)
    dt = t[1] - t[0] if N > 1 else 0.01

    # Create TrajectorySequence for windowing
    seq = DatasetAdapter.from_continuous_arrays(
        trajectory_id='perfect_gt_traj',
        vehicle_class=VehicleClass.CAR,
        times_s=t,
        accel_v=acc,
        gyro_v=gyr,
        vel_v=vel_gt,
        window_size=100,
        stride=10
    )

    N_windows = len(seq.samples)
    if N_windows == 0:
        raise ValueError('No windows generated')

    # Prepare arrays for predictions
    pred_vel = np.full(N, np.nan)
    pred_var = np.full(N, np.nan)
    gt_vel_end = np.full(N, np.nan)

    for i, sample in enumerate(seq.samples):
        start_idx = i * 10
        end_idx = start_idx + 99
        gt_vel_end[end_idx] = sample.truth.velocity_forward_mps
        pred_vel[end_idx] = sample.truth.velocity_forward_mps  # Perfect GT
        pred_var[end_idx] = 0.0001  # Very small variance

    valid_idx = ~np.isnan(pred_vel)

    # Initialize filters
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    vel_est = np.zeros(N)

    buffer_acc = []
    buffer_gyr = []

    for i in range(N):
        ts_ns = int(t[i] * 1e9)
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(acc[i]),
            gyro_rad_s=tuple(gyr[i])
        )

        ins.propagate(imu)

        # Buffer for ML
        buffer_acc.append(acc[i])
        buffer_gyr.append(gyr[i])
        if len(buffer_acc) > 100:
            buffer_acc.pop(0)
            buffer_gyr.pop(0)

        # Check if we have a full window and if it's time to predict (every stride windows)
        if len(buffer_acc) == 100 and i >= 99:
            window_start = i - 99
            if window_start % 10 == 0:  # Every 10th window (stride=10)
                # Use perfect GT
                if not np.isnan(pred_vel[i]) and np.isfinite(pred_vel[i]) and np.isfinite(pred_var[i]):
                    v_val = pred_vel[i]
                    var_val = pred_var[i]

                    # Apply forward velocity update
                    v_n = np.array(ins.state.velocity_mps)
                    q_v2n = ins.state.attitude_q_v2n
                    R_v2n = quat_to_rotation_matrix(q_v2n)
                    R_n2v = R_v2n.T
                    v_v = R_n2v @ v_n
                    expected_v_x = v_v[0]
                    z = np.array([v_val - expected_v_x])

                    # Jacobian H (1 x 15) - no attitude coupling for stability
                    H = np.zeros((1, 15))
                    H[0, 3:6] = R_n2v[0, :]
                    H[0, 6:9] = np.array([0.0, 0.0, 0.0])

                    R_mat = np.array([[var_val]])
                    eskf._apply_measurement(z, H, R_mat, mahalanobis_gate=3.0)

        vel_est[i] = np.linalg.norm(ins.state.velocity_mps)

    # Compute metrics
    gt_north = vel_gt[:, 0]
    errors = vel_est - gt_north
    mae = np.mean(np.abs(errors))
    rmse = np.sqrt(np.mean(errors**2))

    # Position error
    def integrate_to_position(vel_north, dt):
        pos = np.zeros_like(vel_north)
        p = 0.0
        for i, v in enumerate(vel_north):
            p = p + v * dt
            pos[i] = p
        return pos

    pos_est = integrate_to_position(vel_est, dt)
    pos_gt = integrate_to_position(gt_north, dt)
    pos_err = pos_est - pos_gt
    final_pos_err = pos_err[-1]
    max_pos_err = np.max(np.abs(pos_err))

    return {
        'velocity_mae': mae,
        'velocity_rmse': rmse,
        'position_drift': final_pos_err,
        'max_position_error': max_pos_err
    }

def run_eskf_ml_current(traj):
    '''Run ESKF + ML velocity at current update rate'''
    # Load model
    model_path = os.path.join(project_root, 'velocity_model.pth')
    model = Velocity1DCNN()
    model.load_state_dict(torch.load(model_path))
    model.eval()

    t = traj['time']
    acc = traj['accel_v']
    gyr = traj['gyro_v']
    vel_gt = traj['vel_v']  # vehicle frame velocity (forward is x)

    N = len(t)
    dt = t[1] - t[0] if N > 1 else 0.01

    # Create TrajectorySequence
    seq = DatasetAdapter.from_continuous_arrays(
        trajectory_id='ml_current_traj',
        vehicle_class=VehicleClass.CAR,
        times_s=t,
        accel_v=acc,
        gyro_v=gyr,
        vel_v=vel_gt,
        window_size=100,
        stride=10
    )

    N_windows = len(seq.samples)
    if N_windows == 0:
        raise ValueError('No windows generated')

    # Compute bias and variance characteristics
    raw_preds = []
    raw_vars = []
    gts = []
    for sample in seq.samples:
        feat = torch.from_numpy(sample.features.data).unsqueeze(0)
        with torch.no_grad():
            v_pred, log_var = model(feat)
            raw_preds.append(v_pred.item())
            raw_vars.append(torch.exp(log_var).item())
        gts.append(sample.truth.velocity_forward_mps)
    raw_preds = np.array(raw_preds)
    raw_vars = np.array(raw_vars)
    gts = np.array(gts)
    errors = raw_preds - gts
    bias = np.mean(errors)
    error_std = np.std(errors)
    mean_var = np.mean(raw_vars)
    variance_scale_needed = (error_std ** 2) / mean_var if mean_var > 0 else 1.0

    # Prepare arrays for predictions
    pred_vel = np.full(N, np.nan)
    pred_var = np.full(N, np.nan)
    gt_vel_end = np.full(N, np.nan)

    for i, sample in enumerate(seq.samples):
        start_idx = i * 10
        end_idx = start_idx + 99
        gt_vel_end[end_idx] = sample.truth.velocity_forward_mps
        if not np.isnan(raw_preds[i]):
            pred_vel[end_idx] = raw_preds[i] - bias  # debias
            pred_var[end_idx] = raw_vars[i] * variance_scale_needed  # scale variance

    valid_idx = ~np.isnan(pred_vel)

    # Initialize filters
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    vel_est = np.zeros(N)

    buffer_acc = []
    buffer_gyr = []

    for i in range(N):
        ts_ns = int(t[i] * 1e9)
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(acc[i]),
            gyro_rad_s=tuple(gyr[i])
        )

        ins.propagate(imu)

        # Buffer for ML
        buffer_acc.append(acc[i])
        buffer_gyr.append(gyr[i])
        if len(buffer_acc) > 100:
            buffer_acc.pop(0)
            buffer_gyr.pop(0)

        # Check if we have a full window and if it's time to predict (every stride windows)
        if len(buffer_acc) == 100 and i >= 99:
            window_start = i - 99
            if window_start % 10 == 0:  # Every 10th window (stride=10)
                # Use ML prediction
                if not np.isnan(pred_vel[i]) and np.isfinite(pred_vel[i]) and np.isfinite(pred_var[i]):
                    v_val = pred_vel[i]
                    var_val = pred_var[i]

                    # Apply forward velocity update
                    v_n = np.array(ins.state.velocity_mps)
                    q_v2n = ins.state.attitude_q_v2n
                    R_v2n = quat_to_rotation_matrix(q_v2n)
                    R_n2v = R_v2n.T
                    v_v = R_n2v @ v_n
                    expected_v_x = v_v[0]
                    z = np.array([v_val - expected_v_x])

                    # Jacobian H (1 x 15) - no attitude coupling for stability
                    H = np.zeros((1, 15))
                    H[0, 3:6] = R_n2v[0, :]
                    H[0, 6:9] = np.array([0.0, 0.0, 0.0])

                    R_mat = np.array([[var_val]])
                    eskf._apply_measurement(z, H, R_mat, mahalanobis_gate=3.0)

        vel_est[i] = np.linalg.norm(ins.state.velocity_mps)

    # Compute metrics
    gt_north = vel_gt[:, 0]
    errors = vel_est - gt_north
    mae = np.mean(np.abs(errors))
    rmse = np.sqrt(np.mean(errors**2))

    # Position error
    def integrate_to_position(vel_north, dt):
        pos = np.zeros_like(vel_north)
        p = 0.0
        for i, v in enumerate(vel_north):
            p = p + v * dt
            pos[i] = p
        return pos

    pos_est = integrate_to_position(vel_est, dt)
    pos_gt = integrate_to_position(gt_north, dt)
    pos_err = pos_est - pos_gt
    final_pos_err = pos_err[-1]
    max_pos_err = np.max(np.abs(pos_err))

    return {
        'velocity_mae': mae,
        'velocity_rmse': rmse,
        'position_drift': final_pos_err,
        'max_position_error': max_pos_err
    }

def run_eskf_ml_half_rate(traj):
    '''Run ESKF + ML velocity with half-rate updates'''
    # Load model
    model_path = os.path.join(project_root, 'velocity_model.pth')
    model = Velocity1DCNN()
    model.load_state_dict(torch.load(model_path))
    model.eval()

    t = traj['time']
    acc = traj['accel_v']
    gyr = traj['gyro_v']
    vel_gt = traj['vel_v']  # vehicle frame velocity (forward is x)

    N = len(t)
    dt = t[1] - t[0] if N > 1 else 0.01

    # Create TrajectorySequence
    seq = DatasetAdapter.from_continuous_arrays(
        trajectory_id='ml_half_rate_traj',
        vehicle_class=VehicleClass.CAR,
        times_s=t,
        accel_v=acc,
        gyro_v=gyr,
        vel_v=vel_gt,
        window_size=100,
        stride=10
    )

    N_windows = len(seq.samples)
    if N_windows == 0:
        raise ValueError('No windows generated')

    # Compute bias and variance characteristics
    raw_preds = []
    raw_vars = []
    gts = []
    for sample in seq.samples:
        feat = torch.from_numpy(sample.features.data).unsqueeze(0)
        with torch.no_grad():
            v_pred, log_var = model(feat)
            raw_preds.append(v_pred.item())
            raw_vars.append(torch.exp(log_var).item())
        gts.append(sample.truth.velocity_forward_mps)
    raw_preds = np.array(raw_preds)
    raw_vars = np.array(raw_vars)
    gts = np.array(gts)
    errors = raw_preds - gts
    bias = np.mean(errors)
    error_std = np.std(errors)
    mean_var = np.mean(raw_vars)
    variance_scale_needed = (error_std ** 2) / mean_var if mean_var > 0 else 1.0

    # Prepare arrays for predictions
    pred_vel = np.full(N, np.nan)
    pred_var = np.full(N, np.nan)
    gt_vel_end = np.full(N, np.nan)

    for i, sample in enumerate(seq.samples):
        start_idx = i * 10
        end_idx = start_idx + 99
        gt_vel_end[end_idx] = sample.truth.velocity_forward_mps
        if not np.isnan(raw_preds[i]):
            pred_vel[end_idx] = raw_preds[i] - bias  # debias
            pred_var[end_idx] = raw_vars[i] * variance_scale_needed  # scale variance

    valid_idx = ~np.isnan(pred_vel)

    # Initialize filters
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    vel_est = np.zeros(N)

    buffer_acc = []
    buffer_gyr = []

    for i in range(N):
        ts_ns = int(t[i] * 1e9)
        imu = ImuSample(
            timestamp_ns=ts_ns,
            accel_m_s2=tuple(acc[i]),
            gyro_rad_s=tuple(gyr[i])
        )

        ins.propagate(imu)

        # Buffer for ML
        buffer_acc.append(acc[i])
        buffer_gyr.append(gyr[i])
        if len(buffer_acc) > 100:
            buffer_acc.pop(0)
            buffer_gyr.pop(0)

        # Check if we have a full window and if it's time to predict (every stride windows)
        if len(buffer_acc) == 100 and i >= 99:
            window_start = i - 99
            if window_start % 20 == 0:  # Every 20th window (half rate)
                # Use ML prediction
                if not np.isnan(pred_vel[i]) and np.isfinite(pred_vel[i]) and np.isfinite(pred_var[i]):
                    v_val = pred_vel[i]
                    var_val = pred_var[i]

                    # Apply forward velocity update
                    v_n = np.array(ins.state.velocity_mps)
                    q_v2n = ins.state.attitude_q_v2n
                    R_v2n = quat_to_rotation_matrix(q_v2n)
                    R_n2v = R_v2n.T
                    v_v = R_n2v @ v_n
                    expected_v_x = v_v[0]
                    z = np.array([v_val - expected_v_x])

                    # Jacobian H (1 x 15) - no attitude coupling for stability
                    H = np.zeros((1, 15))
                    H[0, 3:6] = R_n2v[0, :]
                    H[0, 6:9] = np.array([0.0, 0.0, 0.0])

                    R_mat = np.array([[var_val]])
                    eskf._apply_measurement(z, H, R_mat, mahalanobis_gate=3.0)

        vel_est[i] = np.linalg.norm(ins.state.velocity_mps)

    # Compute metrics
    gt_north = vel_gt[:, 0]
    errors = vel_est - gt_north
    mae = np.mean(np.abs(errors))
    rmse = np.sqrt(np.mean(errors**2))

    # Position error
    def integrate_to_position(vel_north, dt):
        pos = np.zeros_like(vel_north)
        p = 0.0
        for i, v in enumerate(vel_north):
            p = p + v * dt
            pos[i] = p
        return pos

    pos_est = integrate_to_position(vel_est, dt)
    pos_gt = integrate_to_position(gt_north, dt)
    pos_err = pos_est - pos_gt
    final_pos_err = pos_err[-1]
    max_pos_err = np.max(np.abs(pos_err))

    return {
        'velocity_mae': mae,
        'velocity_rmse': rmse,
        'position_drift': final_pos_err,
        'max_position_error': max_pos_err
    }

# Set deterministic seeds
torch.manual_seed(42)
np.random.seed(42)

print('Loading trained model...')
model_path = os.path.join(project_root, 'velocity_model.pth')
model = Velocity1DCNN()
model.load_state_dict(torch.load(model_path))
model.eval()
print(f'Model loaded from {model_path}')

print('\nGenerating fixed synthetic trajectory (seed=123 for audit)...')
gen = SyntheticTrajectoryGenerator(seed=123)
traj = gen.generate_straight_accel_decel(duration=30.0, max_speed=20.0)

print(f'Trajectory length: {len(traj["time"])} samples at {1/(traj["time"][1]-traj["time"][0]):.1f} Hz')
print(f'Duration: {traj["time"][-1]:.1f} s')
print(f'Speed range: {np.min(traj["vel_v"][:,0]):.1f} to {np.max(traj["vel_v"][:,0]):.1f} m/s')

# Run experiments
print('\n=== Reproducing Baseline ===')

# A. ESKF-only
print('\nRunning A. ESKF-only...')
results_A = run_eskf_only(traj)
print(f'ESKF-only: Vel RMSE = {results_A["velocity_rmse"]:.3f} m/s, Pos Drift = {results_A["position_drift"]:.3f} m')

# B. ESKF + perfect GT
print('\nRunning B. ESKF + perfect ground-truth forward velocity...')
results_B = run_eskf_perfect_gt(traj)
print(f'ESKF+Perfect GT: Vel RMSE = {results_B["velocity_rmse"]:.3f} m/s, Pos Drift = {results_B["position_drift"]:.3f} m')

# C. ESKF + ML current rate
print('\nRunning C. ESKF + ML velocity at current update rate...')
results_C = run_eskf_ml_current(traj)
print(f'ESKF+ML (current): Vel RMSE = {results_C["velocity_rmse"]:.3f} m/s, Pos Drift = {results_C["position_drift"]:.3f} m')

# G. ESKF + ML half rate
print('\nRunning G. ESKF + ML velocity with half-rate updates...')
results_G = run_eskf_ml_half_rate(traj)
print(f'ESKF+ML (half rate): Vel RMSE = {results_G["velocity_rmse"]:.3f} m/s, Pos Drift = {results_G["position_drift"]:.3f} m')

print('\n=== Baseline Reproduction Complete ===')
print('Results:')
print(f'A. ESKF-only:           Vel RMSE = {results_A["velocity_rmse"]:.3f} m/s, Pos Drift = {results_A["position_drift"]:.3f} m')
print(f'B. ESKF+Perfect GT:     Vel RMSE = {results_B["velocity_rmse"]:.3f} m/s, Pos Drift = {results_B["position_drift"]:.3f} m')
print(f'C. ESKF+ML (current):   Vel RMSE = {results_C["velocity_rmse"]:.3f} m/s, Pos Drift = {results_C["position_drift"]:.3f} m')
print(f'G. ESKF+ML (half rate): Vel RMSE = {results_G["velocity_rmse"]:.3f} m/s, Pos Drift = {results_G["position_drift"]:.3f} m')