import numpy as np

A = np.array([[0 for _ in range(7)] for _ in range(6)])
v = np.array([0.0] * 7).reshape(7, 1)
print(A.shape, v.shape)
print(A @ v)