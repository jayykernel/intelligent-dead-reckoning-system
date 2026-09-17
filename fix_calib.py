import re

with open('engine/calibration/calibrator.py', 'r') as f:
    content = f.read()

new_func = '''    def calibrate_from_session(self, acc: np.ndarray, gyro: np.ndarray, speed: np.ndarray, dt: float = 0.1) -> bool:
        """
        Run calibration on a segment of data (e.g., first 60 seconds).
        acc: (N, 3) 
        gyro: (N, 3)
        speed: (N,) true or GPS speed
        
        Returns True if successful, False if insufficient dynamic data.
        """
        N = len(acc)
        
        # Detect stationary using a mix of speed and IMU variance
        # Two-wheelers have high engine idling vibration, so acc_var can be up to 0.5
        acc_mag = np.linalg.norm(acc, axis=1)
        window = int(1.0 / dt)
        
        acc_var = np.array([np.var(acc_mag[i:i+window]) for i in range(N-window + 1)])
        padded_acc_var = np.pad(acc_var, (window//2, N - len(acc_var) - window//2), mode='edge')
        
        # Criteria for stationary: speed < 0.5 m/s AND not experiencing crazy motion (acc_var < 0.5)
        # Or, if speed is not available/reliable, use rigid IMU threshold
        stationary_mask = (speed < 0.5) & (padded_acc_var < 0.5)
        
        if np.sum(stationary_mask) < 10:
             # Try relaxing for very noisy two-wheeler idling
             stationary_mask = (speed < 1.0) & (padded_acc_var < 1.0)
             
        if np.sum(stationary_mask) < 10:
             print(f"  WARNING: Not enough stationary data (detected {np.sum(stationary_mask)}).")
             return False
             
        stat_acc = acc[stationary_mask]
        stat_gyro = gyro[stationary_mask]
        
        self.gyro_bias = np.mean(stat_gyro, axis=0)
        
        g_phone = np.mean(stat_acc, axis=0)
        z_v_phone = g_phone / np.linalg.norm(g_phone)
        
        # 2. Isolate forward acceleration periods for Forward Axis
        # Compute speed derivative
        ds = np.gradient(speed, dt)
        
        # Look for strong forward acceleration
        accel_mask = ds > 0.2
        if np.sum(accel_mask) < 10:
             accel_mask = ds > 0.1
             
        if np.sum(accel_mask) < 10:
            print("  WARNING: Not enough strong forward acceleration to identify vehicle Y-axis.")
            return False
            
        fwd_acc = acc[accel_mask] - g_phone
        
        # Average the forward acceleration in phone frame
        a_fwd_phone = np.mean(fwd_acc, axis=0)
        
        # Project onto the horizontal plane (orthogonal to Z)
        y_v_phone = a_fwd_phone - np.dot(a_fwd_phone, z_v_phone) * z_v_phone
        norm_y = np.linalg.norm(y_v_phone)
        if norm_y < 0.05:
            print("  WARNING: Forward acceleration is co-linear with gravity or too weak. Cannot align.")
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
        
        return True'''

pattern = r'    def calibrate_from_session\(self, acc: np\.ndarray, gyro: np\.ndarray, speed: np\.ndarray, dt: float = 0\.1\) -> bool:.*?return True'
content = re.sub(pattern, new_func, content, flags=re.DOTALL)

with open('engine/calibration/calibrator.py', 'w') as f:
    f.write(content)
