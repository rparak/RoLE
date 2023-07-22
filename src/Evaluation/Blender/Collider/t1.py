import numpy as np

n_joints = np.array([1,2,3,4,5,6])


for i, n_i in enumerate(n_joints):
    for _, n_ij in enumerate(n_joints[(i + 2)::]):
        print(n_i, n_ij)

