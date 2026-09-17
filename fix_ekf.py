import re

with open("engine/fusion/ekf.py", "r") as f:
    code = f.read()

# Fix F matrix
code = code.replace("F[0:3, 9:12] = 0.5 * R * dt**2", "F[0:3, 9:12] = -0.5 * R * dt**2")
code = code.replace("F[3:6, 9:12] = R * dt", "F[3:6, 9:12] = -R * dt")

# Fix inject_error_state
injection_old = """        qw, qx, qy, qz = self.q
        dw, dx, dy, dz = dq
        self.q = np.array([
            qw*dw - qx*dx - qy*dy - qz*dz,
            qw*dx + qx*dw + qy*dz - qz*dy,
            qw*dy - qx*dz + qy*dw + qz*dx,
            qw*dz + qx*dy - qy*dx + qz*dw
        ])"""

# global rotation dq * q:
injection_new = """        qw, qx, qy, qz = self.q
        dw, dx, dy, dz = dq # dq is in NAV frame! So q_new = dq * q
        self.q = np.array([
            dw*qw - dx*qx - dy*qy - dz*qz,
            dw*qx + dx*qw + dy*qz - dz*qy,
            dw*qy - dx*qz + dy*qw + dz*qx,
            dw*qz + dx*qy - dy*qx + dz*qw
        ])"""

code = code.replace(injection_old, injection_new)

with open("engine/fusion/ekf.py", "w") as f:
    f.write(code)
