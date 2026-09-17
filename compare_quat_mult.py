import numpy as np
from engine.strapdown import quat_mult, rotvec_to_quat

q = np.array([0.7071, 0.1, 0.2, 0.3])
rot_vec = np.array([0.01, -0.02, 0.05])

# strapdown dq
dq_strap = rotvec_to_quat(rot_vec)
q_strap = quat_mult(q, dq_strap)

# ekf dq
angle = np.linalg.norm(rot_vec)
axis = rot_vec / angle
half_angle = angle / 2.0
sin_half = np.sin(half_angle)
dq_ekf = np.array([np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half])
qw, qx, qy, qz = q
dw, dx, dy, dz = dq_ekf
q_ekf = np.array([
    qw*dw - qx*dx - qy*dy - qz*dz,
    qw*dx + qx*dw + qy*dz - qz*dy,
    qw*dy - qx*dz + qy*dw + qz*dx,
    qw*dz + qx*dy - qy*dx + qz*dw
])

print("q_strap:", q_strap)
print("q_ekf:  ", q_ekf)
print("diff:   ", np.abs(q_strap - q_ekf).max())
