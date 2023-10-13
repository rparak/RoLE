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
#   ../Lib/Transformation/Core
import Lib.Transformation.Core as Transformation
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics
#   ../Configuration/Parameters
import Configuration.Parameters
#   ..
import Lib.Interpolation.Utilities

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

    # Get the actual and desired tool center point (TCP).
    T_actual  = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_0, 'Fast', Robot_Str)[1]
    T_desired = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]

    # ...
    t = np.linspace(Configuration.Parameters.CONST_T_0, Configuration.Parameters.CONST_T_1, 100)

    theta_0 = abs_j_pos_0.copy(); 
    for _, t_i in enumerate(t):
        # ...
        p_i = Lib.Interpolation.Utilities.Lerp('Explicit', T_actual.p.all(), T_desired.p.all(), t_i)

        # Obtain the spherical linear interpolation (Slerp) between the given quaternions.
        q_i = Lib.Interpolation.Utilities.Slerp('Quaternion', T_actual.Get_Rotation('QUATERNION'), 
                                              T_desired.Get_Rotation('QUATERNION'), t_i)
        
        # Express the homogeneous transformation matrix of an object from position and rotation.
        TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(q_i.all(), 'QUATERNION').Translation(p_i)

        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (_, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, theta_0, Robot_Str, 'Best')
        
        # Obtain the last absolute position of the joint.
        theta_0 = theta.copy()

    # Get the observed tool center point (TCP) to check the results.
    T_observed = Lib.Kinematics.Core.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]

    # Check that the calculation has been performed successfully.
    accuracy = Mathematics.Euclidean_Norm((T_observed - T_desired).all())
    if Mathematics.Euclidean_Norm((T_observed - T_desired).all()) <= 1e-5:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')

if __name__ == '__main__':
    main()