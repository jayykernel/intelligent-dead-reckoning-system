import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from engine.fusion.fusion_engine import FusionEngine

def debug_vta28():
    data = np.load('data/processed/Vta28_processed.npz')
    
    # Init fusion
    fusion = FusionEngine(dt=0.1, vehicle_type='car')
    
    # Run subset up to end of outage (167.8s to 227.8s)
    # Give it ~30s of prep time 
    start_idx = int((167.8 - 30) // 0.1)
    end_idx = int((227.8 + 10) // 0.1)
    
    # Extract data
    acc = data['acc_veh'][start_idx:end_idx]
    gyro = data['gyro_veh'][start_idx:end_idx]
    speed = data['speed'][start_idx:end_idx]
    ai_speed = data['ai_speed'][start_idx:end_idx]
    ai_std = data['ai_speed_std'][start_idx:end_idx]
    mag = data['mag_veh'][start_idx:end_idx] if 'mag_veh' in data else None
    gt = data['gt_xy'][start_idx:end_idx] if 'gt_xy' in data else None
    dr_flag = data['dr_flag'][start_idx:end_idx] if 'dr_flag' in data else np.zeros_like(speed)
    
    # State tracking
    pos_history = []
    heading_history = []
    
    for i in range(len(acc)):
        fusion.process_imu(acc[i], gyro[i])
        
        # Check ZUPT
        if dr_flag[i] == 0:
            fusion.process_gnss(gt[i], speed[i], np.array([2.0, 2.0]))
        else:
            fusion.process_ai_speed(ai_speed[i], ai_std[i])
            if fusion.constrained_ins:
                # Apply NHC wrapper via EKF directly for diag
                v_veh = fusion.ekf.q_to_rot(fusion.ekf.q).T @ fusion.ekf.v
                v_constrained = fusion.constrained_ins.constrain(acc[i], gyro[i], v_veh, 'car')
                v_nav_expected = fusion.ekf.q_to_rot(fusion.ekf.q) @ v_constrained
                meas_z = v_nav_expected - fusion.ekf.v
                cov_R = np.eye(3) * 0.1
                H = np.zeros((3, 15))
                H[0:3, 3:6] = np.eye(3)
                fusion.ekf.update(meas_z, H, cov_R)
        
        if mag is not None:
             fusion.process_mag(mag[i])
             
        pos_history.append(fusion.get_position())
        # Heading from unit quat
        qw, qx, qy, qz = fusion.ekf.q
        h = np.arctan2(2.0*(qx*qy + qw*qz), 1.0 - 2.0*(qx**2 + qz**2))
        heading_history.append(h)
        
    pos = np.array(pos_history)
    h_hist = np.array(heading_history)
        
    print(f"Final pos: {pos[-1]}")
    if gt is not None:
        print(f"GT pos at end: {gt[-1]}")
        print(f"Error magnitude: {np.linalg.norm(pos[-1] - gt[-1]):.2f}m")

debug_vta28()
