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

Robot_Str = Parameters.ABB_IRB_120_L_Ax_Str

# ...
J = Kinematics.Get_Geometric_Jacobian(Robot_Str.Theta.Home, Robot_Str)

print(np.round(J, 5))