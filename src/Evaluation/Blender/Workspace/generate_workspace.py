# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# OS (Operating system interfaces)
import os
# Custom Script:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Utilities/File_IO
import Lib.Utilities.File_IO
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters

"""
Description:
    Initialization of constants.
"""
# The name of the input file and folder.
CONST_FILE_NAME = 'tool0_workspace_data'

def main():
    """
    Description:
        A program for visualization the workspace of an individual robot structure.

        Note:
            The structures of the robot are defined below:
                ../Parameters/Robot.py
    """

    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Kinematics')[0] + 'Kinematics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = Parameters.ABB_IRB_120_Str

    # Example: 
    #   Robot workspace dataset (x, y, z).
    data = Lib.Utilities.File_IO.Load(f'{project_folder}/src/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME}', 'txt', ',')
    
    # Generate a simplified convex polyhedron from the input data (x, y, z).
    Lib.Blender.Utilities.Generate_Convex_Polyhedron_From_Data(f'{Robot_Str.Name}_Workspace', data, {'RGBA': [0.0, 1.0, 0.0, 1.0], 'alpha': 1.0})
    
    # The wireframe modifier transforms a mesh object into a wireframe model with a defined size (thickness).
    #Lib.Blender.Utilities.Transform_Object_To_Wireframe(f'{Robot_Str.Name}_Workspace', 0.005)

if __name__ == '__main__':
    main()