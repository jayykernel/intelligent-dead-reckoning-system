import numpy as np
from engine.fusion.ekf import ErrorStateEKF

q = np.array([1, 0, 0, 0])
dq = np.array([np.cos(0.1), np.sin(0.1), 0, 0])
# q * dq in EKF:
qw, qx, qy, qz = q
dw, dx, dy, dz = dq
q_new1 = np.array([
    qw*dw - qx*dx - qy*dy - qz*dz,
    qw*dx + qx*dw + qy*dz - qz*dy,
    qw*dy - qx*dz + qy*dw + qz*dx,
    qw*dz + qx*dy - qy*dx + qz*dw
])
print(q_new1)
