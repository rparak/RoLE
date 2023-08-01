# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Library:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/URDF/Core
import Lib.URDF.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str

def main():
    """
    Description:
        ...

        More information can be found here:
            ../Lib/URDF/Core.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Industrial_Robots_Kinematics')[0] + 'Industrial_Robots_Kinematics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # ...
    Lib.URDF.Core.Generate_URDF(Robot_Str, True, f'{project_folder}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}')

    # display ...

if __name__ == "__main__":
    sys.exit(main())