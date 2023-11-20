# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' +  'src')
# OS (Operating system interfaces)
import os
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Workspace/Core
import RoLE.Workspace.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str

def main():
    """
    Description:
        A program to generate the workspace of a robot arm from the absolute positions of the joints using the Monte Carlo method.

        Note:
            Absolute joint positions are generated from the program below:
                ../Workspace/gen_abs_joint_pos.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Input File: Absolute orientation of the joints.
    CONST_FILE_NAME_IN = 'abs_joint_pos_data'
    print(f'[INFO] Path (Input File): {project_folder}/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_IN}.txt')
    # Output File: X, Y, Z positions of the workspace.
    CONST_FILE_NAME_OUT = 'tool0_workspace_data'
    print(f'[INFO] Path (Output File): {project_folder}/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_OUT}.txt')

    # Generate x, y, z positions of the workspace from the absolute positions
    # of the joints.
    RoLE.Workspace.Core.Generate_Workspace_XYZ(Robot_Str, f'{project_folder}/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_IN}.txt', 
                                              f'{project_folder}/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_OUT}.txt')
    
if __name__ == "__main__":
    sys.exit(main())
