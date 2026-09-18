import numpy as np
from engine.fusion.ekf import ErrorStateEKF

ekf = ErrorStateEKF()
# set a random orientation
q = np.array([0.7071, 0.0, 0.0, 0.7071]) # 90 deg yaw
R = ekf.quat_to_rot(q)

def get_yaw(R_mat):
    return np.arctan2(R_mat[0, 1], R_mat[1, 1])

psi0 = get_yaw(R)
eps = 1e-6

# Perturb delta_theta_z
dtheta = np.array([0, 0, eps])
# Using the exact same formula as in ekf.py inject_error_state
axis = dtheta / eps
half = eps / 2.0
dq = np.array([np.cos(half), axis[0]*np.sin(half), axis[1]*np.sin(half), axis[2]*np.sin(half)])

# q_new = q * dq (body-frame rotation)
qw, qx, qy, qz = q
dw, dx, dy, dz = dq
q_new = np.array([
    qw*dw - qx*dx - qy*dy - qz*dz,
    qw*dx + qx*dw + qy*dz - qz*dy,
    qw*dy - qx*dz + qy*dw + qz*dx,
    qw*dz + qx*dy - qy*dx + qz*dw
])
R_new = ekf.quat_to_rot(q_new)
psi_new = get_yaw(R_new)

num_jac = (psi_new - psi0) / eps
print(f"Numerical Jacobian d(psi)/d(delta_theta_z): {num_jac:.6f}")
print(f"H[0, 8] in ekf.py: -1.0")

