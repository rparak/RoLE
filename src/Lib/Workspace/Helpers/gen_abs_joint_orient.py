# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Platform (Platform identification data)
import platform
# System (Default)
import sys
if platform.system() == 'Windows':
    # Windows Path.
    sys.path.append('..\\..\\..\\..')
else:
    # Linux (Ubuntu) Path.
    sys.path.append('../../../..' + 'src') 
# Functools (Operation of other functions)
import functools
# Operator (Standard operators as functions)
import operator
# Custom Library:
#   ../Lib/Manipulator/Parameters (The main parameters of the manipulator)
import Lib.Manipulator.Parameters as Parameters
#   ../Lib/Manipulator/Workspace/Core
import Lib.Manipulator.Workspace.Core

def main():
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = Parameters.ABB_IRB_120_Structure

    # Number of samples for joint orientation combinations to generate 
    # the workspace of a robotic arm.
    nCr_Joint = Lib.Manipulator.Workspace.Core.Get_Number_of_Samples(Robot_Str.Name)

    # The name of the resulting file.
    print(f'[INFO] Path: ../../../../../Data/Workspace/{Robot_Str.Name}/abs_joint_orient_data.txt')

    theta = list([[Lib.Manipulator.Workspace.Core.CONST_NONE_VALUE]] * 7)
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
    #C:\Users\romanp\Documents\GitHub\Blender_Robotics\Data\Workspace
    Lib.Manipulator.Workspace.Core.Generate_Absolute_Joint_Orientation(f'../../../../../Data/Workspace/{Robot_Str.Name}/abs_joint_orient_data.txt', 
                                                                       NUM_OF_COMBINATIONS, theta)

if __name__ == "__main__":
    sys.exit(main())