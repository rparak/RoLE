# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Time (Time access and conversions)
import time
import numpy as np
# ...
import Lib.Kinematics.Core as Kinematics
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters

Robot_Str = Parameters.EPSON_LS3_B401S_Str

# ...
J = Kinematics.Get_Geometric_Jacobian(Robot_Str.Theta.Zero, Robot_Str)