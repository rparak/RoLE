# System (Default)
import sys
#   Add access if it is not in the system path.
sys.path.append('../../..')
# Custom Library:
#   ../Lib/Manipulator/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Manipulator/Parameters
import Lib.Parameters.Robot as Parameters
# ...
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

import numpy as np

def main():
    Robot_Str = CONST_ROBOT_TYPE

    Robot_Str.T.Zero_Cfg = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

    for i in range(len(Robot_Str.Theta.Zero)):
        if i == 0:
            T_i = Robot_Str.T.Base.Inverse() @ Robot_Str.T.Zero_Cfg[i]
        else:
            T_i = Robot_Str.T.Zero_Cfg[i - 1].Inverse() @ Robot_Str.T.Zero_Cfg[i]

        # Get the translational and rotational part from the transformation matrix.
        p = T_i.p; Euler_Angles = T_i.Get_Rotation('ZYX')

        print(f'[INFO] Joint_{i}:')
        print(f'[INFO] >> p: [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]')
        print(f'[INFO] >> Euler Angles: [{Euler_Angles.x:.3f}, {Euler_Angles.y:.3f}, {Euler_Angles.z:.3f}]')

        #del T_i
    
if __name__ == '__main__':
    main()
