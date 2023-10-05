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

import Lib.Transformation.Core as Transformation

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
    
    # Set printing options.
    np.set_printoptions(suppress=True, precision=5)

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the homogeneous transformation matrix of the robot end-effector from the input absolute joint positions.
    #   FK: 
    #       Theta --> T
    TCP_Position = Lib.Kinematics.Core.Forward_Kinematics(np.array([Mathematics.Degree_To_Radian(25.0), Mathematics.Degree_To_Radian(-20.0), 0.15, Mathematics.Degree_To_Radian(15.0)],
                                                                   dtype = np.float64), 'Fast', Robot_Str)[1]

    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Numerical(TCP_Position, Robot_Str.Theta.Zero, 'Newton-Raphson', Robot_Str, 
                                                                     {'num_of_iteration': 100, 'tolerance': 1e-9})

    # Display results.
    print(f'[INFO] The solution was successfully found: {info["successful"]}')
    print(f'[INFO] >> position_err = {info["error"]["position"]}, orientation_err = {info["error"]["orientation"]}')
    print(f'[INFO] >> theta = {theta}')

if __name__ == '__main__':
    main()