import numpy as np

a = np.array([True, False, False], dtype=bool)
b = np.array([False, True, True], dtype=bool)

print(a | b)