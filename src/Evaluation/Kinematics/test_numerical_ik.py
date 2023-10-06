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
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Configuration/Parameters
import Configuration.Parameters

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

    # Initialization of the class to generate trajectory.
    Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=0.05)

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # Generation of multi-axis position trajectories from input parameters.
    theta_arr = []
    for _, (th_actual, th_desired) in enumerate(zip(abs_j_pos_0, abs_j_pos_1)):
        (theta_arr_i, _, _) = Polynomial_Cls.Generate(th_actual, th_desired, 0.0, 0.0, 0.0, 0.0,
                                                      Configuration.Parameters.CONST_T_0, Configuration.Parameters.CONST_T_1)
        theta_arr.append(theta_arr_i)

    # Calculation of inverse kinematics (IK) using the chosen numerical method.
    theta_0 = abs_j_pos_0.copy(); theta_T = np.array(theta_arr, dtype=np.float64).T
    for _, theta_arr_i in enumerate(theta_T):
        # Obtain the homogeneous transformation matrix of the robot end-effector from the input absolute joint positions.
        #   FK: 
        #       Theta --> T
        TCP_Position = Lib.Kinematics.Core.Forward_Kinematics(theta_arr_i, 'Fast', Robot_Str)[1]

        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (info, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Numerical(TCP_Position, theta_0, 'Newton-Raphson', Robot_Str, 
                                                                        {'num_of_iteration': 100, 'tolerance': 1e-10})
        
        # Check the calculation.
        if info["successful"] == False:
            break

        # Obtain the last absolute position of the joint.
        theta_0 = theta.copy()

    # Check that the calculation has been performed successfully.
    if (theta == abs_j_pos_1).all():
        print('[INFO] The IK solution test was successful.')
    else:
        print('[WARNING] A problem occurred during the calculation.')

if __name__ == '__main__':
    main()