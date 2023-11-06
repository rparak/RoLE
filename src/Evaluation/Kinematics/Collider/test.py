# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# ...
import Lib.Transformation.Utilities.Mathematics as Mathematics

a = np.array([1, 2, 3, 4, 5], dtype=np.int8)

offset = 0; n = 0
for i, Collider_i in enumerate(a):
    for j, Collider_j in enumerate(a[(i + 1) + offset::], 
                                    start=(i + 1) + offset):
        #print(Collider_i, Collider_j)
        n += 1

print(n)
print(Mathematics.Combinations(a.size - offset, 2))

