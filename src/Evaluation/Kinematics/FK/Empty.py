# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

def main():
    """
    [-0.  0. -0.]
    [-90.   0. -90.]
    [-90.   0. -90.]
    [ 45.  90. 135.]
    [-90.   0. -90.]
    [ 45.  90. -45.]
    """
    Robot_Str = Parameters.ABB_IRB_120_Str
    Robot_Str.T.Base = HTM_Cls(None, np.float32).Rotation(np.deg2rad([0.0, 0.0, 20.0]), 'ZYX')
    Robot_Str.T.Zero_Cfg = Kinematics.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

    print('Joint 0')
    T_0 = Robot_Str.T.Zero_Cfg[0] @ Robot_Str.T.Base.Inverse()
    print(np.round(T_0.Get_Rotation('ZYX').Degree, 2))
    print(np.round(Robot_Str.T.Zero_Cfg[0].Get_Rotation('ZYX').Degree, 2))
    print('Joint 1')
    T_1 = (Robot_Str.T.Zero_Cfg[1] @ T_0.Inverse())
    print(np.round(T_1.Get_Rotation('ZYX').Degree, 2))
    print(np.round(Robot_Str.T.Zero_Cfg[1].Get_Rotation('ZYX').Degree, 2))
    print('Joint 2')
    T_2 = T_1.Inverse() @ (Robot_Str.T.Base.Inverse() @ Robot_Str.T.Zero_Cfg[2])
    print(np.round(T_2.Get_Rotation('ZYX').Degree, 2))
    print(np.round(Robot_Str.T.Zero_Cfg[1].Get_Rotation('ZYX').Degree, 2))

if __name__ == '__main__':
    sys.exit(main())