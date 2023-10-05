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
        A program to calculate the inverse kinematics (IK) of the RRPR robotic structure (called SCARA) using an analytical method.

        Two methods can be used to obtain IK solutions: 'All' or 'Best'.
            1\ 'All': Obtain the all possible solutions.
            2\ 'Best': Automatically obtain the best solution.
    """
    
    # Set printing options.
    #np.set_printoptions(suppress=True, precision=5)

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the homogeneous transformation matrix of the robot end-effector from the input absolute joint positions.
    #   FK: 
    #       Theta --> T
    TCP_Position = Lib.Kinematics.Core.Forward_Kinematics(np.array([Mathematics.Degree_To_Radian(25.0), Mathematics.Degree_To_Radian(-20.0), 0.1, Mathematics.Degree_To_Radian(15.0)],
                                                                   dtype = np.float64), 'Fast', Robot_Str)[1]
    
    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (error, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, Robot_Str.Theta.Home, Robot_Str, 'All')

    # Display results.
    for i, (err_i, th_i) in enumerate(zip(error.values(), theta)):
        print(f'[INFO] Solution {i}:')
        print(f'[INFO] >> position_err = {err_i[0]}, orientation_err = {err_i[1]}')
        print(f'[INFO] >> theta = {th_i}')

if __name__ == '__main__':
    main()