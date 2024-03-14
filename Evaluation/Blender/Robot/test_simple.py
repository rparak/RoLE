# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' +  'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: 
#   Blender
#       ../RoLE/Blender/Parameters/Camera
import Blender.Parameters.Camera
#       ../RoLE/Blender/Utilities
import Blender.Utilities
#       ../RoLE/Blender/Robot/Core
import Blender.Robot.Core
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

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
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Blender.Parameters.Camera.Right_View_Camera_Parameters_Str
# The properties of the robot structure in the Blender environment.
CONST_PROPERTIES = {'fps': 100, 'visibility': {'Viewpoint_EE': False, 'Colliders': False, 
                                               'Workspace': False, 'Ghost': False}}
# If the value is 'True', the homogeneous transformation matrix of the robot 
# base will be obtained from the Blender environment.
CONST_USE_BLENDER_ROBOT_BASE = False
# Animation stop(t_0), start(t_1) time in seconds.
CONST_T_0 = 0.0
CONST_T_1 = 2.0

def main():
    """
    Description:
        A program that demonstrates how to work with a specific class to control a robotic arm in Blender scene.
    """
    
    # Deselect all objects in the current scene.
    Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if Blender.Utilities.Object_Exist('Camera'):
        Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Modification of the robot base.
    if CONST_USE_BLENDER_ROBOT_BASE == True:
        if 'ABB_IRB_14000' in Robot_Str.Name:
            Robot_Str.T.Base @= HTM_Cls(bpy.data.objects['ABB_IRB_14000_ID_001'].matrix_basis, 
                                    np.float64)
        else:
            Robot_Str.T.Base = HTM_Cls(bpy.data.objects[f'{Robot_Str.Parameters.Name}_ID_{Robot_Str.Parameters.Id:03}'].matrix_basis, 
                                    np.float64)  

    # Initialization of the class to work with a robotic arm object in a Blender scene.
    Robot_ID_0_Cls = Blender.Robot.Core.Robot_Cls(Robot_Str, CONST_PROPERTIES)
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    # Reset the absolute position of the robot joints to the 'Zero'.
    Robot_ID_0_Cls.Reset('Zero')
    
    # Get the FPS (Frames Per Seconds) value from the Blender settings.
    fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base

    # The first frame on which the animation starts.
    bpy.context.scene.frame_start = np.int32(CONST_T_0 * fps)

    print('[INFO] Absolute Joint Positions (desired):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Parameters.Theta.Home):
        print(f'[INFO] >> Joint_{i}({th_i + 0.0:.3f})')

    # Set the absolute position of the robot joints.
    Robot_ID_0_Cls.Set_Absolute_Joint_Position(Robot_ID_0_Cls.Parameters.Theta.Home, CONST_T_0, CONST_T_1)

    # The last frame on which the animation stops.
    bpy.context.scene.frame_end = np.int32(CONST_T_1 * fps)

    # Get the the absolute positions of the robot's joints.
    print('[INFO] Absolute Joint Positions (actual):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Theta):
        print(f'[INFO] >> Joint_{i}({th_i + 0.0:.3f})')

    # Get the homogeneous transformation matrix of the robot end-effector. Parameters position 
    # and orientation (euler angles).
    print('[INFO] Tool Center Point (TCP):')
    print(f'[INFO] >> p: x({Robot_ID_0_Cls.T_EE.p.x + 0.0:.3f}), y({Robot_ID_0_Cls.T_EE.p.y + 0.0:.3f}), z({Robot_ID_0_Cls.T_EE.p.z + 0.0:.3f})')
    Euler_Angles = Robot_ID_0_Cls.T_EE.Get_Rotation('ZYX') + [0.0, 0.0, 0.0]
    print(f'[INFO] >> Euler Angles: x({Euler_Angles.x:.3f}), y({Euler_Angles.y:.3f}), z({Euler_Angles.z:.3f})')
    
if __name__ == '__main__':
    main()
