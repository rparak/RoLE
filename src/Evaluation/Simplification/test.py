# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Kinematics/Utilities/Forward_Kinematics
import Lib.Kinematics.Utilities.Forward_Kinematics

"""
Description:
    Initialization of constants.
"""
# Number of randomly generated samples.
CONST_SIZE = 1

def main():
    """
    Description:
        A program to compare several approaches to calculate forward kinematics for an individual robotic arm. The program 
        also checks the correctness of the calculation.
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = Parameters.ABB_IRB_120_Str


    th = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    """
    Description:
        Time evaluation of different calculation methods.
    """
    print('[INFO] The calculation is in progress.')

    T_Standard = Lib.Kinematics.Core.Forward_Kinematics(th, 'Standard', Robot_Str)[1]
    T_Modified = Lib.Kinematics.Core.Forward_Kinematics(th, 'Modified', Robot_Str)[1]

    print(f'[INFO] Forward Kinematics (Standard)')
    print(np.round(T_Standard.all(), 3))
    print(f'[INFO] Forward Kinematics (Modified)')
    print(np.round(T_Modified.all(), 3))


if __name__ == '__main__':
    sys.exit(main())