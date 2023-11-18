# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Blender/Utilities
import RoLE.Blender.Utilities
#   ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Blender/Parameters/Camera
import RoLE.Blender.Parameters.Camera

"""
Description:
    Open {robot_name}.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/RoLE/Blender/Robot
        $ blender {robot_name}.blend

    Note:
        Where the variable 'robot_name' is the name of the controlled robot to be used.
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_R_Str

def main():
    """
    Description:
        A program for placing collision objects on individual parts of a robotic structure.

        Collision objects are generated from the script, see below:
            ./Collider/gen_colliders.py
    """
    
    # Deselect all objects in the current scene.
    RoLE.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    RoLE.Blender.Utilities.Remove_Animation_Data()

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Set the structure of the main parameters of the controlled robot.
    Robot_ID_0_Cls = RoLE.Blender.Core.Robot_Cls(Robot_Str, {'Viewpoint_EE': False, 'Colliders': True, 
                                                             'Workspace': False})

    # Reset the absolute position of the robot joints to the 'Zero'.
    Robot_ID_0_Cls.Reset('Zero')
    
    # Set the collision object transformation of the base robot.
    RoLE.Blender.Utilities.Set_Object_Transformation(f'Base_Collider_{Robot_Str.Name}_ID_{Robot_Str.Id:03}', Robot_Str.T.Base)
    
    # Get the configuration of the homogeneous matrix of each joint using forward kinematics.
    #   Note:
    #       The absolute position of the joints must be the same as in the Robot_Cls reset function at the top.
    T_Arr = RoLE.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]
    
    # Places collision objects on the remaining parts of the robot.
    for _, (th_name_i, T_i) in enumerate(zip(Robot_Str.Theta.Name, T_Arr)):
        # Get the joint ID from the string.
        id = th_name_i.removesuffix(f'_{Robot_Str.Name}_ID_{Robot_Str.Id:03}').removeprefix('Joint_')

        # Set the collision object transformation of the robot joint.
        RoLE.Blender.Utilities.Set_Object_Transformation(f'Joint_{id}_Collider_{Robot_Str.Name}_ID_{Robot_Str.Id:03}', T_i)
    
if __name__ == '__main__':
    main()