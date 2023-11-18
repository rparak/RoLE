# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# OS (Operating system interfaces)
import os
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Blender/Utilities
import RoLE.Blender.Utilities
#   ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
#   ../RoLE/Parameters/Robot
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
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str
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
    RoLE.Blender.Utilities.Deselect_All()

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Kinematics')[0] + 'Kinematics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Example: 
    #   Robot workspace dataset (x, y, z).
    data = RoLE.Utilities.File_IO.Load(f'{project_folder}/src/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME}', 'txt', ',')
    
    # Generate a simplified convex polyhedron from the input data (x, y, z).
    RoLE.Blender.Utilities.Generate_Convex_Polyhedron_From_Data(f'Workspace_{Robot_Str.Name}_ID_{Robot_Str.Id:03}', data, {'RGBA': [0.0, 1.0, 0.0, 1.0], 'alpha': 1.0})
    
    # Set the transparency of the object material.
    RoLE.Blender.Utilities.Set_Object_Material_Transparency(f'Workspace_{Robot_Str.Name}_ID_{Robot_Str.Id:03}', 0.05)

if __name__ == '__main__':
    main()