import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.nhc_zupt.constrained_ins import ConstrainedINS

# Test _is_stopped with different thresholds
acc_veh = np.array([0.0, 0.0, 9.81 + 0.1])
gyro_veh = np.array([0.01, 0.01, 0.01])
v_veh = np.array([0.1, 0.1, 0.0])
ai_speed = 0.1

ins = ConstrainedINS()

print(f"Original _is_stopped: {ins._is_stopped(acc_veh, gyro_veh, v_veh, ai_speed)}")

# Adjust thresholds to be more permissive
class FixedConstrainedINS(ConstrainedINS):
    def _is_stopped(self, acc_veh: np.ndarray, gyro_veh: np.ndarray, v_veh: np.ndarray = None, ai_speed: float = None, vehicle_type: str = 'car') -> bool:
        acc_mag = np.linalg.norm(acc_veh)
        gyro_mag = np.linalg.norm(gyro_veh)
        
        # Test: more permissive for car
        acc_thresh = 0.3
        gyro_thresh = 0.1
        imu_quiet = (abs(acc_mag - self.g) < acc_thresh) and (gyro_mag < gyro_thresh)
        
        if v_veh is not None:
             v_mag = np.linalg.norm(v_veh)
             return imu_quiet and (v_mag < 0.5)
        return imu_quiet

fixed_ins = FixedConstrainedINS()
print(f"Permissive _is_stopped: {fixed_ins._is_stopped(acc_veh, gyro_veh, v_veh, ai_speed)}")
