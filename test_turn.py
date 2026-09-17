import numpy as np
from engine.strapdown import dcm_to_quat, rotvec_to_quat, quat_mult, quat_to_dcm

psi = np.radians(30.0)
R_yaw = np.array([
    [np.cos(psi), np.sin(psi), 0.0],
    [-np.sin(psi), np.cos(psi), 0.0],
    [0.0, 0.0, 1.0]
])
q0 = dcm_to_quat(R_yaw)

# Turn right by 10 deg:
d_psi = np.radians(10.0)
# Gyro Z in vehicle frame when turning right (clockwise) in right-handed ENU (X right, Y forward, Z up):
# Turning right means yaw angle decreases in math convention (or clockwise rotation).
# In vehicle frame, turning right means angular velocity vector points down (-Z).
rot_vec = np.array([0.0, 0.0, -d_psi])

dq = rotvec_to_quat(rot_vec)
q1 = quat_mult(q0, dq)
R1 = quat_to_dcm(q1)

v_veh_fwd = np.array([0, 1, 0])
v_nav_fwd = R1 @ v_veh_fwd

new_heading = np.degrees(np.arctan2(v_nav_fwd[0], v_nav_fwd[1]))
print(f"Initial heading: 30 deg")
print(f"Turned right by 10 deg -> New heading: {new_heading:.2f} deg (expected 40 deg)")
