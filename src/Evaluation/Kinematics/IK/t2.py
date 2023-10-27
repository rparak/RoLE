import numpy as np

# https://github.com/Walid-khaled/7DOF-KUKA-Linear-Axis-Forward-and-Inverse-Kinematics/blob/main/Jacobian.m
A = np.array([[0 for _ in range(7)] for _ in range(6)])
V = np.eye(6)
v = np.array([0.0] * 6).reshape(6, 1)
print(A.shape, v.shape, V.shape)
#print(np.linalg.pinv(A) @ v)
#print(A.T @ v)
#print(A.T @ V @ v)

T = np.eye(4)
print(np.divide(T, T))