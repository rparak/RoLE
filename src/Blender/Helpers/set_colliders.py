# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Custom Script:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Blender/Parameters/Camera
import Lib.Blender.Parameters.Camera

"""
Description:
    Open {robot_name}.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/Open_Industrial_Robotics/Blender/Robot
        $ blender {robot_name}.blend

    Note:
        Where the variable 'robot_name' is the name of the controlled robot to be used.
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_L_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Lib.Blender.Parameters.Camera.Right_View_Camera_Parameters_Str

def main():
    """
    Description:
        A program for placing collision objects on individual parts of a robotic structure.

        Collision objects are generated from the script, see below:
            ./Collder/gen_colliders.py
    """
    
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if Lib.Blender.Utilities.Object_Exist('Camera'):
        Lib.Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Set the structure of the main parameters of the controlled robot.
    Robot_ID_0_Cls = Lib.Blender.Core.Robot_Cls(Robot_Str, {'Viewpoint_EE': False, 'Colliders': False, 
                                                            'Workspace': False})

    # Reset the absolute position of the robot joints to the 'Zero'.
    Robot_ID_0_Cls.Reset('Zero')
    
    # Set the collision object transformation of the base robot.
    Lib.Blender.Utilities.Set_Object_Transformation(f'Base_Collider', Robot_Str.T.Base)
    
    # Get the configuration of the homogeneous matrix of each joint using forward kinematics.
    #   Note:
    #       The absolute position of the joints must be the same as in the Robot_Cls reset function at the top.
    T_Arr = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)
    
    # Places collision objects on the remaining parts of the robot.
    for _, (th_name_i, T_i) in enumerate(zip(Robot_Str.Theta.Name, T_Arr)):
        # Get the joint ID from the string.
        id = th_name_i.removesuffix(f'_{Robot_Str.Name}_ID_{Robot_Str.Id:03}').removeprefix('Joint_')

        # Set the collision object transformation of the robot joint.
        Lib.Blender.Utilities.Set_Object_Transformation(f'Joint_{id}_Collider', T_i)
    
if __name__ == '__main__':
    main()