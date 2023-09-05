import numpy as np

a = np.array([1, 2, 3, 4, 5, 6]); offset = 1
for i, a_i in enumerate(a):
    for _, a_j in enumerate(a[(i + 1) + offset::]):
        print(a_i, a_j)