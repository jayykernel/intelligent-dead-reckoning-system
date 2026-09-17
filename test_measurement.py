import numpy as np
from scipy.stats import chi2

P = np.eye(15) * 0.1
H = np.zeros((3, 15))
H[0:3, 0:3] = np.eye(3)
R_cov = np.eye(3) * 9.0

# 1. Innovation vector
y = np.array([5.0, 5.0, 0.0]) # delta of 5m

S = H @ P @ H.T + R_cov
S = 0.5 * (S + S.T)

S_inv = np.linalg.inv(S)
nis = float(y.T @ S_inv @ y)

chi2_thresh = float(chi2.ppf(0.99, df=3))
print("S diag:", np.diag(S))
print("NIS:", nis)
print("Thresh:", chi2_thresh)

