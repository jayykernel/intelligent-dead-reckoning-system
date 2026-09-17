import numpy as np
from engine.fusion.ekf import ErrorStateEKF

f_nav = np.array([0.1, 0.2, 0.0])
R = np.eye(3)
dt = 0.1
skew = ErrorStateEKF.skew_symmetric(f_nav)

# Old F[0:3, 6:9]
F_06_old = -0.5 * skew * dt**2
# F[3:6, 6:9]
F_36_old = -skew * dt

print("F_06_old:\n", F_06_old)
print("F_36_old:\n", F_36_old)

# Check F[0:3, 6:9] in EKF (if new one)
print("F[0,8] should be -0.5 * f_x? No, skew matrix")
print("Skew:\n", skew)
