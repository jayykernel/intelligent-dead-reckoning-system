import numpy as np
from engine.strapdown import StrapdownINS
from engine.fusion.ekf import ErrorStateEKF

p0 = np.zeros(3)
v0 = np.zeros(3)
q0 = np.array([1.0, 0.0, 0.0, 0.0])

strap = StrapdownINS()
strap.initialize(p0, v0, q0=q0)

ekf = ErrorStateEKF(dt=0.1)
ekf.set_initial_state(p0, v0, q0)

acc = np.array([0.1, 0.2, 9.80665])
gyro = np.array([0.01, 0.02, 0.03])

p_s, v_s, q_s = strap.step(acc, gyro, 0.1)
ekf.predict(acc, gyro, dt=0.1)

print("Strap pos:", p_s)
print("EKF pos:  ", ekf.p)
print("Strap vel:", v_s)
print("EKF vel:  ", ekf.v)
print("Strap quat:", q_s)
print("EKF quat:  ", ekf.q)
