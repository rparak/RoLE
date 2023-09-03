# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Functools (Operation of other functions)
import functools
# Operator (Standard operators as functions)
import operator
# Custom Library:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Workspace/Core
import Lib.Workspace.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str

def main():
    """
    Description:
        A program to generate the absolute positions of the joints of a robotic arm.

        Note:
            More information can be found here:
                ../Lib/Wokrspace/Core.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Open_Industrial_Robotics')[0] + 'Open_Industrial_Robotics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Number of samples for joint orientation combinations to generate 
    # the workspace of a robotic arm.
    nCr_Joint = Lib.Workspace.Core.Get_Number_of_Samples(Robot_Str.Name)

    # The name of the resulting file.
    print(f'[INFO] Path: {project_folder}/src/Data/Workspace/{Robot_Str.Name}/abs_joint_pos_data.txt')

    theta = list([[Lib.Workspace.Core.CONST_NONE_VALUE]] * 7)
    print('[INFO] Input Data:')
    for i, th_limit in enumerate(Robot_Str.Theta.Limit):
        print(f'[INFO]  Theta {i}:')
        if nCr_Joint[i] != 0:
            # Generation of evenly spaced numbers in the specified interval.
            theta[i] = np.linspace(th_limit[0], th_limit[1], nCr_Joint[i], 
                                   dtype=np.float32)
            print(f'[INFO]   - Limit (-): {theta[i][0]} | Limit (+): {theta[i][-1]}')
        else:
            theta[i] = np.array([nCr_Joint[i]])
            print(f'[INFO]   - Limit (-): {theta[i]} | Limit (+): {theta[i]}')

        print(f'[INFO]   - Number of samples: {len(theta[i])}')
        print(f'[INFO]   - Data Type: {np.float32}')

    # Get the number of combinations.
    NUM_OF_COMBINATIONS = functools.reduce(operator.mul, map(len, theta[0:len(Robot_Str.Theta.Limit)]), 1)
    print(f'[INFO] Number of combinations between {len(theta)} lists of elements: {NUM_OF_COMBINATIONS}')

    # Generate the absolute orientation of the joints to calculate the robot's workspace 
    # and save this data to a file.
    Lib.Workspace.Core.Generate_Absolute_Joint_Orientation(f'{project_folder}/src/Data/Workspace/{Robot_Str.Name}/abs_joint_pos_data.txt', 
                                                           NUM_OF_COMBINATIONS, theta)

if __name__ == "__main__":
    sys.exit(main())