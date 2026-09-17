import numpy as np
from engine.strapdown import quat_to_dcm, rotvec_to_quat, quat_mult
from engine.fusion.ekf import ErrorStateEKF

q = np.array([0.7071, 0.1, 0.2, 0.3])
q = q / np.linalg.norm(q)

R_strap = quat_to_dcm(q)
R_ekf = ErrorStateEKF.quat_to_rot(q)

print("R_strap:\n", R_strap)
print("R_ekf:\n", R_ekf)
print("Diff:\n", np.abs(R_strap - R_ekf).max())
