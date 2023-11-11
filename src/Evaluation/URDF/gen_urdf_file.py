# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Custom Lib.: Industrial Robotics Library for Everyone (IRLE)
#   ../IRLE/Parameters/Robot
import IRLE.Parameters.Robot as Parameters
#   ../IRLE/URDF/Core
import IRLE.URDF.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str
# Use a mesh to represent a visual/collision object.
CONST_ENABLE_MESH = True

def main():
    """
    Description:
        A program to generate and save the URDF (Unified Robotics Description Format) file for a defined robot structure.

        More information can be found here:
            ../IRLE/URDF/Core.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Open_Industrial_Robotics')[0] + 'Open_Industrial_Robotics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialization of a class for working with URDF.
    URDF_Generator_Cls_0 = IRLE.URDF.Core.URDF_Generator_Cls(Robot_Str, CONST_ENABLE_MESH, [0.90, 0.90, 0.90, 1.0])
    #   Generate and save the URDF.
    URDF_Generator_Cls_0.Generate()
    #URDF_Generator_Cls_0.Save(f'{project_folder}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}')

if __name__ == "__main__":
    sys.exit(main())