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

def run_eskf_ml_with_policy(traj, update_every_n_samples=1, bias_and_var_from_heldout=False):
    '''Run ESKF + ML velocity with a given update policy.

    update_every_n_samples: apply ML update every n samples (after buffer is full).
                            1 means every sample, 2 means every other sample, etc.
    bias_and_var_from_heldout: if True, compute bias and variance scaling from held-out seed (666)
                               and apply to the audit trajectory (seed=123).
                               If False, compute from the audit trajectory itself.
    '''
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

    # Compute bias and variance characteristics
    if bias_and_var_from_heldout:
        # Use held-out seed (666) to compute bias and variance scaling
        gen_heldout = SyntheticTrajectoryGenerator(seed=666)
        traj_heldout = gen_heldout.generate_straight_accel_decel(duration=30.0, max_speed=20.0)
        seq_heldout = DatasetAdapter.from_continuous_arrays(
            trajectory_id='heldout_traj',
            vehicle_class=VehicleClass.CAR,
            times_s=traj_heldout['time'],
            accel_v=traj_heldout['accel_v'],
            gyro_v=traj_heldout['gyro_v'],
            vel_v=traj_heldout['vel_v'],
            window_size=100,
            stride=10
        )
        raw_preds_heldout = []
        raw_vars_heldout = []
        gts_heldout = []
        for sample in seq_heldout.samples:
            feat = torch.from_numpy(sample.features.data).unsqueeze(0)
            with torch.no_grad():
                v_pred, log_var = model(feat)
                raw_preds_heldout.append(v_pred.item())
                raw_vars_heldout.append(torch.exp(log_var).item())
            gts_heldout.append(sample.truth.velocity_forward_mps)
        raw_preds_heldout = np.array(raw_preds_heldout)
        raw_vars_heldout = np.array(raw_vars_heldout)
        gts_heldout = np.array(gts_heldout)
        errors_heldout = raw_preds_heldout - gts_heldout
        bias = np.mean(errors_heldout)
        error_std = np.std(errors_heldout)
        mean_var = np.mean(raw_vars_heldout)
        variance_scale_needed = (error_std ** 2) / mean_var if mean_var > 0 else 1.0
    else:
        # Use the audit trajectory itself to compute bias and variance seeding
        seq = DatasetAdapter.from_continuous_arrays(
            trajectory_id='char_traj',
            vehicle_class=VehicleClass.CAR,
            times_s=t,
            accel_v=acc,
            gyro_v=gyr,
            vel_v=vel_gt,
            window_size=100,
            stride=10
        )
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

    # Initialize filters
    ins = StrapdownINS(create_initial_state())
    eskf = ErrorStateKalmanFilter(ins)

    vel_est = np.zeros(N)

    # Buffer for ML estimator (simulating VelocityEstimatorAPI)
    buffer_acc = []
    buffer_gyr = []
    # Counter for update policy
    samples_since_last_update = 0

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

        # Check if we have a full window
        if len(buffer_acc) == 100:
            # Compute ML prediction
            feat = torch.from_numpy(np.array(buffer_acc).T).unsqueeze(0)  # shape (1, 6, 100)
            with torch.no_grad():
                v_pred, log_var = model(feat)
                v_pred = v_pred.item() - bias  # debias
                var_val = torch.exp(log_var).item() * variance_scale_needed  # scale variance

            # Apply update policy: only update every n samples
            samples_since_last_update += 1
            if samples_since_last_update >= update_every_n_samples:
                # Apply forward velocity update
                v_n = np.array(ins.state.velocity_mps)
                q_v2n = ins.state.attitude_q_v2n
                R_v2n = quat_to_rotation_matrix(q_v2n)
                R_n2v = R_v2n.T
                v_v = R_n2v @ v_n
                expected_v_x = v_v[0]
                z = np.array([v_pred - expected_v_x])

                # Jacobian H (1 x 15) - no attitude coupling for stability
                H = np.zeros((1, 15))
                H[0, 3:6] = R_n2v[0, :]
                H[0, 6:9] = np.array([0.0, 0.0, 0.0])

                R_mat = np.array([[var_val]])
                eskf._apply_measurement(z, H, R_mat, mahalanobis_gate=3.0)

                samples_since_last_update = 0  # reset counter

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
print('\n=== Testing ML Aiding Policies ===')

# A. ESKF-only (baseline)
print('\nRunning A. ESKF-only...')
results_A = run_eskf_only(traj)
print(f'ESKF-only:           Vel RMSE = {results_A["velocity_rmse"]:.3f} m/s, Pos Drift = {results_A["position_drift"]:.3f} m')

# B. ESKF + ML with update every sample (100 Hz) - overlapping windows, update every sample
print('\nRunning B. ESKF + ML velocity update every sample (100 Hz)...')
results_B = run_eskf_ml_with_policy(traj, update_every_n_samples=1, bias_and_var_from_heldout=False)
print(f'ESKF+ML (100 Hz):    Vel RMSE = {results_B["velocity_rmse"]:.3f} m/s, Pos Drift = {results_B["position_drift"]:.3f} m')

# C. ESKF + ML with update every 10 samples (10 Hz) - overlapping windows, update every 10 samples
print('\nRunning C. ESKF + ML velocity update every 10 samples (10 Hz)...')
results_C = run_eskf_ml_with_policy(traj, update_every_n_samples=10, bias_and_var_from_heldout=False)
print(f'ESKF+ML (10 Hz):     Vel RMSE = {results_C["velocity_rmse"]:.3f} m/s, Pos Drift = {results_C["position_drift"]:.3f} m')

# D. ESKF + ML with update every 20 samples (5 Hz) - overlapping windows, update every 20 samples
print('\nRunning D. ESKF + ML velocity update every 20 samples (5 Hz)...')
results_D = run_eskf_ml_with_policy(traj, update_every_n_samples=20, bias_and_var_from_heldout=False)
print(f'ESKF+ML (5 Hz):      Vel RMSE = {results_D["velocity_rmse"]:.3f} m/s, Pos Drift = {results_D["position_drift"]:.3f} m')

# E. ESKF + ML with update every sample (100 Hz) but bias and variance from held-out seed
print('\nRunning E. ESKF + ML velocity update every sample (100 Hz) with held-out bias/var...')
results_E = run_eskf_ml_with_policy(traj, update_every_n_samples=1, bias_and_var_from_heldout=True)
print(f'ESKF+ML (100 Hz, held-out bias/var): Vel RMSE = {results_E["velocity_rmse"]:.3f} m/s, Pos Drift = {results_E["position_drift"]:.3f} m')

# F. ESKF + ML with update every 20 samples (5 Hz) and bias and variance from held-out seed
print('\nRunning F. ESKF + ML velocity update every 20 samples (5 Hz) with held-out bias/var...')
results_F = run_eskf_ml_with_policy(traj, update_every_n_samples=20, bias_and_var_from_heldout=True)
print(f'ESKF+ML (5 Hz, held-out bias/var):   Vel RMSE = {results_F["velocity_rmse"]:.3f} m/s, Pos Drift = {results_F["position_drift"]:.3f} m')

print('\n=== Policy Test Complete ===')
print('Results:')
print(f'A. ESKF-only:                            Vel RMSE = {results_A["velocity_rmse"]:.3f} m/s, Pos Drift = {results_A["position_drift"]:.3f} m')
print(f'B. ESKF+ML (100 Hz overlap):           Vel RMSE = {results_B["velocity_rmse"]:.3f} m/s, Pos Drift = {results_B["position_drift"]:.3f} m')
print(f'C. ESKF+ML (10 Hz overlap):            Vel RMSE = {results_C["velocity_rmse"]:.3f} m/s, Pos Drift = {results_C["position_drift"]:.3f} m')
print(f'D. ESKF+ML (5 Hz overlap):             Vel RMSE = {results_D["velocity_rmse"]:.3f} m/s, Pos Drift = {results_D["position_drift"]:.3f} m')
print(f'E. ESKF+ML (100 Hz, held-out bias/var): Vel RMSE = {results_E["velocity_rmse"]:.3f} m/s, Pos Drift = {results_E["position_drift"]:.3f} m')
print(f'F. ESKF+ML (5 Hz, held-out bias/var):  Vel RMSE = {results_F["velocity_rmse"]:.3f} m/s, Pos Drift = {results_F["position_drift"]:.3f} m')

# Determine if any policy passes the acceptance criterion:
# Primary: not degrade navigation relative to ESKF-only (i.e., position drift magnitude <= ESKF-only magnitude? or just not worse?)
# We'll consider a policy passes if the absolute position drift is less than or equal to that of ESKF-only (58.207 m) AND the velocity RMSE is less than or equal to that of ESKF-only (2.501 m/s).
# Actually, the requirement is: "not degrade navigation relative to ESKF-only". We'll interpret as not worse in both metrics.
print('\n=== Acceptance Check (not degrading ESKF-only) ===')
for label, res in [('B', results_B), ('C', results_C), ('D', results_D), ('E', results_E), ('F', results_F)]:
    drift_ok = abs(res['position_drift']) <= abs(results_A['position_drift'])  # not larger magnitude
    vel_ok = res['velocity_rmse'] <= results_A['velocity_rmse']  # not higher RMSE
    if drift_ok and vel_ok:
        print(f'Policy {label}: PASSES (drift {res["position_drift"]:.3f} m vs {results_A["position_drift"]:.3f} m, RMSE {res["velocity_rmse"]:.3f} vs {results_A["velocity_rmse"]:.3f})')
    else:
        print(f'Policy {label}: FAILS  (drift {res["position_drift"]:.3f} m vs {results_A["position_drift"]:.3f} m, RMSE {res["velocity_rmse"]:.3f} vs {results_A["velocity_rmse"]:.3f})')