import numpy as np

n_joints = np.array([1,2,3,4])

offset = 0
for i, n_i in enumerate(n_joints):
    for _, n_ij in enumerate(n_joints[(i + offset)::]):
        print(n_i, n_ij)

