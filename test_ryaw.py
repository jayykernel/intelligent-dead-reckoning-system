import numpy as np

psi = np.radians(30.0) # 30 deg clockwise from North
R_yaw = np.array([
    [np.cos(psi), np.sin(psi), 0.0],
    [-np.sin(psi), np.cos(psi), 0.0],
    [0.0, 0.0, 1.0]
])

# If vehicle is level, R_level = I, R0 = R_yaw
v_veh_fwd = np.array([0, 1, 0]) # Vehicle Y is forward
v_nav_fwd = R_yaw @ v_veh_fwd

print(f"v_nav_fwd from R_yaw: {v_nav_fwd}")
# In ENU: East is [1, 0, 0], North is [0, 1, 0].
# 30 deg heading from North means:
# East = sin(30) = 0.5
# North = cos(30) = 0.866
print(f"Expected ENU (East, North): [{np.sin(psi):.4f}, {np.cos(psi):.4f}, 0.0]")
