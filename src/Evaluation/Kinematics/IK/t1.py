# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics
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
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # ...
    T_desired = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]

    # Obtain the spherical linear interpolation (Slerp) between the given quaternions.
    #q = Utilities.Slerp('Quaternion', q_0, q_1, t_i)
    
    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    # Calculation of inverse kinematics (IK) using the analytical method.
    theta_0 = abs_j_pos_0.copy(); theta_T = np.array(theta_arr, dtype=np.float64).T
    for _, theta_arr_i in enumerate(theta_T):

        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (_, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, theta_0, Robot_Str, 'Best')
        
        # Obtain the last absolute position of the joint.
        theta_0 = theta.copy()

    t = time.time() - t_0
    print(f'[INFO] Time: {t:0.05f} in seconds.')

    # Get the actual and desired tool center point (TCP) to check the results.
    T_desired = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]
    T_actual  = Lib.Kinematics.Core.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]

    # Check that the calculation has been performed successfully.
    accuracy = Mathematics.Euclidean_Norm((T_actual - T_desired).all())
    if Mathematics.Euclidean_Norm((T_actual - T_desired).all()) <= 1e-5:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')

if __name__ == '__main__':
    main()