import numpy as np

A = np.array([[0 for _ in range(7)] for _ in range(6)])
v = np.array([0.0] * 6).reshape(6, 1)
print(A.shape, v.shape)
print(np.linalg.pinv(A) @ v)