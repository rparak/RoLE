# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# Custom Lib.:
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
# Numerical IK Parameters.
#   Method.
#       'Newton-Raphson', 'Gauss-Newton', 'Levenberg-Marquardt'
CONST_NIK_METHOD = 'Newton-Raphson'
#   Minimum required tolerance.
CONST_NIK_TOLERANCE = 1e-10

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
    T_0 = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_0, 'Fast', Robot_Str)[1]
    T_1 = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]
    #  ...
    p_0 = T_0.p.all(); p_1 = T_1.p.all()
    #  ...
    q_0 = T_0.Get_Rotation('QUATERNION'); q_1 = T_1.Get_Rotation('QUATERNION')

    # ...
    t = np.arange(Configuration.Parameters.CONST_T_0, Configuration.Parameters.CONST_T_1 + 0.1, 0.1)

    theta_0 = abs_j_pos_0.copy(); 
    for _, t_i in enumerate(t):
        # ...
        p_i = Lib.Interpolation.Utilities.Lerp('Explicit', p_0, p_1, t_i)

        # Obtain the spherical linear interpolation (Slerp) between the given quaternions.
        q_i = Lib.Interpolation.Utilities.Slerp('Quaternion', q_0, q_1, t_i)
        
        # Express the homogeneous transformation matrix of an object from position and rotation.
        T_i = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(q_i.all(), 'QUATERNION').Translation(p_i)

        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (info, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Numerical(T_i, theta_0, CONST_NIK_METHOD, Robot_Str, 
                                                                        {'num_of_iteration': 1000, 'tolerance': CONST_NIK_TOLERANCE})

        # Check the calculation.
        if info["successful"] == False:
            break

        # Obtain the last absolute position of the joint.
        theta_0 = theta.copy()

    # Get the observed tool center point (TCP) to check the results.
    T = Lib.Kinematics.Core.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]

    # Check that the calculation has been performed successfully.
    accuracy = Mathematics.Euclidean_Norm((T - T_1).all())
    if Mathematics.Euclidean_Norm((T - T_1).all()) <= 1e-5:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')

if __name__ == '__main__':
    main()