# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core


"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str

def main():
    """
    Description:
        ...
    """
    
    # ...
    np.set_printoptions(suppress=True, precision=5)

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # ...
    TCP_Position = Lib.Kinematics.Core.Forward_Kinematics(np.array([Mathematics.Degree_To_Radian(25.0), Mathematics.Degree_To_Radian(-20.0), 0.05, Mathematics.Degree_To_Radian(15.0)],
                                                                   dtype = np.float32), 'Modified', Robot_Str)[1]
    
    print(TCP_Position.Get_Rotation('ZYX'))
    # ..
    theta = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, Robot_Str.Theta.Home, Robot_Str, 'All')
    
    print(theta[0])
    print(theta[1])
if __name__ == '__main__':
    main()