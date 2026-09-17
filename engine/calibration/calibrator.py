import numpy as np
from typing import Dict, Tuple, Optional

class CalibrationEngine:
    """
    Estimates Phone-to-Vehicle frame alignment and IMU biases.
    
    Vehicle frame convention (Right-Handed):
    X: Right
    Y: Forward
    Z: Up
    
    Returns:
    - R_phone_to_veh: 3x3 rotation matrix
    - gyro_bias: (3,) rad/s
    - accel_bias: (3,) m/s^2 (optional/future, right now assumed mean gravity minus 9.81 on Z after alignment, but standard is just 0 since gravity provides Z anyway, but scale could be estimated. For this phase, gyro bias is key)
    """
    
    def __init__(self):
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.R_phone_to_veh = np.eye(3)
        
        self.is_calibrated = False
        self.alignment_score = 0.0
        
    def calibrate_from_session(self, acc: np.ndarray, gyro: np.ndarray, speed: np.ndarray, dt: float = 0.1) -> bool:
        """
        Run calibration on a segment of data (e.g., first 60 seconds).
        acc: (N, 3) 
        gyro: (N, 3)
        speed: (N,) true or GPS speed
        
        Returns True if successful, False if insufficient dynamic data.
        """
        N = len(acc)
        
        # 1. Isolate stationary periods for Gyro Bias and Gravity Down
        stationary_mask = speed < 0.2
        if np.sum(stationary_mask) < 20: # Need at least 2 seconds (assuming 10Hz) for reliable bias
            # Not enough stationary data, relax threshold
            stationary_mask = speed < 0.5
            
        if np.sum(stationary_mask) < 10:
            print("Warning: Not enough stationary data for accurate gyro bias / gravity alignment.")
            return False
            
        stat_acc = acc[stationary_mask]
        stat_gyro = gyro[stationary_mask]
        
        self.gyro_bias = np.mean(stat_gyro, axis=0)
        
        g_phone = np.mean(stat_acc, axis=0)
        
        # Vehicle Z (Up) is opposite to gravity
        z_v_phone = g_phone / np.linalg.norm(g_phone)
        
        # 2. Isolate forward acceleration periods for Forward Axis
        # Compute speed derivative
        ds = np.gradient(speed, dt)
        
        # Look for strong forward acceleration
        accel_mask = ds > 0.5  # accelerating at > 0.5 m/s^2
        if np.sum(accel_mask) < 10:
            print("Warning: Not enough strong forward acceleration to identify vehicle Y-axis.")
            return False
            
        fwd_acc = acc[accel_mask] - g_phone
        
        # Average the forward acceleration in phone frame
        a_fwd_phone = np.mean(fwd_acc, axis=0)
        
        # Project onto the horizontal plane (orthogonal to Z)
        y_v_phone = a_fwd_phone - np.dot(a_fwd_phone, z_v_phone) * z_v_phone
        norm_y = np.linalg.norm(y_v_phone)
        if norm_y < 1e-3:
            print("Warning: Forward acceleration is co-linear with gravity. Cannot align.")
            return False
            
        y_v_phone = y_v_phone / norm_y
        
        # 3. Compute X (Right)
        x_v_phone = np.cross(y_v_phone, z_v_phone)
        
        # 4. Construct Rotation Matrix
        R_veh_to_phone = np.column_stack([x_v_phone, y_v_phone, z_v_phone])
        self.R_phone_to_veh = R_veh_to_phone.T
        
        # Score is based on amount of data
        self.alignment_score = min(1.0, np.sum(accel_mask) / 50.0)
        self.is_calibrated = True
        
        return True
        
    def apply(self, acc: np.ndarray, gyro: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply calibration and rotation to turn raw phone IMU into vehicle-frame IMU.
        """
        # Subtract biases
        gyro_corrected = gyro - self.gyro_bias
        acc_corrected = acc - self.accel_bias
        
        # Rotate to vehicle frame
        # For batch N,3
        # R is 3x3, acc_corrected is N,3
        # (R @ a.T).T => a @ R.T
        acc_veh = acc_corrected @ self.R_phone_to_veh.T
        gyro_veh = gyro_corrected @ self.R_phone_to_veh.T
        
        return acc_veh, gyro_veh

    def check_misalignment_trigger(self, acc_window: np.ndarray, speed_window: np.ndarray, dt: float = 0.1, threshold_deg: float = 10.0) -> bool:
        """
        Check if the phone mount has shifted by comparing new data against the existing calibration.
        Returns True if a recalibration should be triggered (misalignment > threshold_deg).
        """
        if not self.is_calibrated:
            return True
            
        N = len(acc_window)
        stationary_mask = speed_window < 0.5
        
        # 1. Check gravity alignment if stationary
        if np.sum(stationary_mask) >= 10:
            stat_acc = acc_window[stationary_mask]
            g_phone = np.mean(stat_acc, axis=0)
            z_v_phone = g_phone / np.linalg.norm(g_phone)
            
            # Existing Z vector from inverse rotation matrix (3rd row)
            old_z_v_phone = self.R_phone_to_veh[2, :]
            
            cos_err = np.dot(z_v_phone, old_z_v_phone)
            err_deg = np.degrees(np.arccos(np.clip(cos_err, -1.0, 1.0)))
            
            if err_deg > threshold_deg:
                return True
                
        # 2. Check forward alignment if accelerating
        ds = np.gradient(speed_window, dt)
        accel_mask = ds > 0.5
        if np.sum(accel_mask) >= 10:
            stat_acc = acc_window[stationary_mask] if np.sum(stationary_mask) > 0 else np.array([0,0,9.81]) @ self.R_phone_to_veh  # hack fallback
            if np.sum(stationary_mask) > 0:
                g_phone = np.mean(stat_acc, axis=0)
            
            fwd_acc = acc_window[accel_mask] - g_phone
            a_fwd_phone = np.mean(fwd_acc, axis=0)
            
            z_v_phone = g_phone / np.linalg.norm(g_phone)
            y_v_phone = a_fwd_phone - np.dot(a_fwd_phone, z_v_phone) * z_v_phone
            norm_y = np.linalg.norm(y_v_phone)
            
            if norm_y > 1e-3:
                y_v_phone = y_v_phone / norm_y
                old_y_v_phone = self.R_phone_to_veh[1, :]
                
                cos_err = np.dot(y_v_phone, old_y_v_phone)
                err_deg = np.degrees(np.arccos(np.clip(cos_err, -1.0, 1.0)))
                
                if err_deg > threshold_deg:
                    return True
                    
        return False
