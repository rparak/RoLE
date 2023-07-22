# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Collider/Utilities
import Lib.Collider.Utilities
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Primitives/Core
from Lib.Primitives.Core import Box_Cls
#   ../Lib.Collider.Core
import Lib.Collider.Core 


def main():
    Size = np.array([1.0, 1.0, 1.0])

    # ...
    Box_i = Box_Cls([0.0, 0.0, 0.0], Size)
    print(Box_i.T.p.all())
    print(Box_i.Size)

    for _, verts_i in enumerate(Box_i.Vertices):
        print(verts_i)

    # ...
    #OBB_i = Lib.Collider.Core.OBB_Cls(Box_i)
    #OBB_i.Transformation(HTM_Cls(None, np.float32))

if __name__ == '__main__':
    main()