# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Configuration/Parameters
import Configuration.Parameters
#   ..Lib/Utilities/File_IO
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
        The same principle applies to the program as for 'test_analytical_ik.py', but this oneis for data collection.
    """
    
    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Open_Industrial_Robotics')[0] + 'Open_Industrial_Robotics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file will be saved.
    file_path = f'{project_folder}/src/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Remove the '*.txt' file if it already exists.
    for _, file_i in enumerate([f'{file_path}/Method_Analytical_IK_TCP',f'{file_path}/Method_Analytical_IK_Absolute_Joint_Positions',
                                f'{file_path}/Method_Analytical_IK_Error']):
      if os.path.isfile(f'{file_i}.txt'):
          os.remove(f'{file_i}.txt')

    # Initialization of the class to generate trajectory.
    Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=0.01)

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # Generation of multi-axis position trajectories from input parameters.
    theta_arr = []
    for _, (th_actual, th_desired) in enumerate(zip(abs_j_pos_0, abs_j_pos_1)):
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
        TCP_Position = Lib.Kinematics.Core.Forward_Kinematics(theta_arr_i, 'Fast', Robot_Str)[1]
        
        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (info, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, theta_0, Robot_Str, 'Best')
        
        # Obtain the last absolute position of the joint.
        theta_0 = theta.copy()

        # Get the translational and rotational part from the transformation matrix.
        p = TCP_Position.p.all(); Quaternions = TCP_Position.Get_Rotation('QUATERNION').all()

        # Save the data to the '*.txt' file.
        File_IO.Save(f'{file_path}/Method_Analytical_IK_TCP', np.append(p, Quaternions), 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Analytical_IK_Absolute_Joint_Positions', theta, 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Analytical_IK_Error', [info['error']['position'], 
                                                                 info['error']['orientation']], 'txt', ',')

    # Display information.
    print(f'[INFO] The files have been successfully saved to the folder:')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_TCP.txt')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_Absolute_Joint_Positions.txt')
    print(f'[INFO] >> {file_path}/Method_Analytical_IK_Error.txt')

if __name__ == '__main__':
    main()