# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' +  'src')
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   Blender
#       ../RoLE/Blender/Utilities
import Blender.Utilities
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters

"""
Description:
    Open Generate.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/RoLE/Blender/Workspace
        $ blender Generate.blend
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# The name of the input file and folder.
CONST_FILE_NAME = 'tool0_workspace_data'

def main():
    """
    Description:
        A program for visualization the workspace of an individual robot structure.

        Note:
            Workspace data is generated from the program below:
                ../Evaluation/Workspace/gen_abs_joint_pos.py
                ../Evaluation/Workspace/gen_xyz_workspace.py
    """

    # Deselect all objects in the current scene.
    Blender.Utilities.Deselect_All()

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Example: 
    #   Robot workspace dataset (x, y, z).
    data = RoLE.Utilities.File_IO.Load(f'{project_folder}/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME}', 'txt', ',')
    
    # Generate a simplified convex polyhedron from the input data (x, y, z).
    Blender.Utilities.Generate_Convex_Polyhedron_From_Data(f'Workspace_{Robot_Str.Name}_ID_{Robot_Str.Id:03}', data, {'RGBA': [0.0, 1.0, 0.0, 1.0], 'alpha': 1.0})
    
    # Set the transparency of the object material.
    Blender.Utilities.Set_Object_Material_Transparency(f'Workspace_{Robot_Str.Name}_ID_{Robot_Str.Id:03}', 0.05)

if __name__ == '__main__':
    main()