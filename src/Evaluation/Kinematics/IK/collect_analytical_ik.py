# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Configuration/Parameters
import Configuration.Parameters
#   ../Lib/Utilities/File_IO
import Lib.Utilities.File_IO as File_IO

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str

def main():
    """
    Description:
        A program to test the calculation of the inverse kinematics (IK) of the RRPR robotic 
        structure, also known as SCARA, using an analytical method.

        Two methods can be used to obtain IK solutions: 'All' or 'Best'.
            1\ 'All': Obtain the all possible solutions.
            2\ 'Best': Automatically obtain the best solution.

        Note:
            The test will be performed by generating a trajectory using a polynomial 
            profile, on which the calculation will be verified.
    """
    
    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Open_Industrial_Robotics')[0] + 'Open_Industrial_Robotics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file will be saved.
    file_path = f'{project_folder}/src/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Remove the '*.txt' file if it already exists.
    for _, file_i in enumerate([f'{file_path}/Method_Analytical_IK_TCP_Desired', 
                                f'{file_path}/Method_Analytical_IK_TCP_Predicted',
                                f'{file_path}/Method_Analytical_IK_Absolute_Joint_Positions', 
                                f'{file_path}/Method_Analytical_IK_Error']):
      if os.path.isfile(f'{file_i}.txt'):
          os.remove(f'{file_i}.txt')

    # Initialization of the class to generate trajectory.
    Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=0.01)

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # Obtain the desired homogeneous transformation matrix T of the tool center point (TCP).
    T_1 = Lib.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]

    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(T_1, abs_j_pos_0, Robot_Str, 'Best')
    
    # Generation of multi-axis position trajectories from input parameters.
    #   Note:
    #       Demonstration of joint interpolation instead of linear interpolation.
    theta_arr = []
    for _, (th_actual, th_desired) in enumerate(zip(abs_j_pos_0, theta)):
        (theta_arr_i, _, _) = Polynomial_Cls.Generate(th_actual, th_desired, 0.0, 0.0, 0.0, 0.0,
                                                      Configuration.Parameters.CONST_T_0, Configuration.Parameters.CONST_T_1)
        theta_arr.append(theta_arr_i)

    print('[INFO] The calculation is in progress.')

    # Calculation of inverse kinematics (IK) using the analytical method.
    theta_0 = abs_j_pos_0.copy(); theta_T = np.array(theta_arr, dtype=np.float64).T
    for _, theta_arr_i in enumerate(theta_T):
        # Obtain the homogeneous transformation matrix of the robot end-effector from the input absolute joint positions.
        #   FK: 
        #       Theta --> T
        T_i = Lib.Kinematics.Core.Forward_Kinematics(theta_arr_i, 'Fast', Robot_Str)[1]
        
        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (info, theta_i) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(T_i, theta_0, Robot_Str, 'Best')

        # Obtain the translation and rotation part from the desired/predicted homogeneous 
        # transformation matrix.
        #   1\ Desired.
        p_i = T_i.p.all(); q_i = T_i.Get_Rotation('QUATERNION').all()
        #   2\ Predicted.
        T_predicted = Lib.Kinematics.Core.Forward_Kinematics(theta_i, 'Fast', Robot_Str)[1]
        p_predicted = T_predicted.p.all(); q_predicted = T_predicted.Get_Rotation('QUATERNION').all()

        # Save the data to the '*.txt' file.
        File_IO.Save(f'{file_path}/Method_Analytical_IK_TCP_Desired', np.append(p_i, q_i), 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Analytical_IK_TCP_Predicted', np.append(p_predicted, q_predicted), 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Analytical_IK_Absolute_Joint_Positions', theta_i, 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Analytical_IK_Error', [info['error']['position'], 
                                                                 info['error']['orientation']], 'txt', ',')
        
        # Obtain the last absolute position of the joint.
        theta_0 = theta_i.copy()

    # Display information.
    print(f'[INFO] The files have been successfully saved to the folder:')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_TCP_Desired.txt')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_TCP_Predicted.txt')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_Absolute_Joint_Positions.txt')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_Error.txt')
    
if __name__ == '__main__':
    sys.exit(main())