# System (Default)
import sys
#   Add access if it is not in the system path.
if 'src' not in sys.path:
    sys.path.append('src/')

#   ../Lib/Primitives/Core
from Lib.Primitives.Core import Box_Cls
#   ../Lib/Primitives/Core
from Lib.Collider.Core import OBB_Cls


OBB_Cls = OBB_Cls(Box_Cls([0.00000, -0.70200, -0.08296], [0.13778, 1.66290, 0.16593]))