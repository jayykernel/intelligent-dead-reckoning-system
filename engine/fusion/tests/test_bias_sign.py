import numpy as np

# Let's verify the derivative of v_next w.r.t b_a
R = np.eye(3)
dt = 0.1
b_a = np.array([0.0, 0.0, 0.0])
acc_raw = np.array([0.0, 0.0, 9.8])
g = np.array([0.0, 0.0, -9.8])

# Nominal integration
acc_corr = acc_raw - b_a
a_nav = R @ acc_corr + g
v_next = a_nav * dt

# Perturb b_a by +eps
eps = 1e-4
b_a_perturbed = np.array([0.0, 0.0, eps])
acc_corr_p = acc_raw - b_a_perturbed
a_nav_p = R @ acc_corr_p + g
v_next_p = a_nav_p * dt

dv = (v_next_p - v_next) / eps
print("Numerical dv/d(b_a):", dv)
print("Theoretical dv/d(b_a) = -R * dt:", -R[:, 2] * dt)
print("Current EKF F[3:6, 9:12] uses: +R * dt =", R[:, 2] * dt)

