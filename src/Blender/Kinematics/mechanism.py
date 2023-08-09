# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Blender/Parameters/Camera
import Lib.Blender.Parameters.Camera
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Blender/Core
import Lib.Blender.Core
#   ../Lib/Parameters/Mechanism
import Lib.Parameters.Mechanism as Parameters

"""
Description:
    Open {mechanism_name}.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/Industrial_Robots_Kinematics/Blender/Mechanism
        $ blender {mechanism_name}.blend

    Note:
        Where the variable 'mechanism_name' is the name of the controlled mechanism to be used.
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled mechanism.
CONST_MECHANISM_0_1_TYPE = Parameters.SMC_LEFB25_14000_0_1_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Lib.Blender.Parameters.Camera.Right_View_Camera_Parameters_Str
# Animation stop(t_0), start(t_1) time in seconds.
CONST_T_0 = 0.0
CONST_T_1 = 2.0

def main():
    """
    Description:
        A program that demonstrates how to work with a specific class to control a mechanism in Blender scene.
    """
    
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if Lib.Blender.Utilities.Object_Exist('Camera'):
        Lib.Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)

    # Set the structure of the main parameters of the controlled mechanism.
    Mechanism_ID_0_1_Cls = Lib.Blender.Core.Mechanism_Cls(CONST_MECHANISM_0_1_TYPE, {'Viewpoint_EE': False, 'Colliders': False})
    print(f'[INFO] Mechanism Name: {Mechanism_ID_0_1_Cls.Parameters.Name}_ID_{Mechanism_ID_0_1_Cls.Parameters.Id:03}')
    
    # Reset the absolute position of the mechanism joints to the 'Zero'.
    Mechanism_ID_0_1_Cls.Reset('Zero')
    
    # Get the FPS (Frames Per Seconds) value from the Blender settings.
    fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base

    # The first frame on which the animation starts.
    bpy.context.scene.frame_start = np.int32(CONST_T_0 * fps)

    print('[INFO] Absolute Joint Positions (desired):')
    print(f'[INFO] >> Joint_0({Mechanism_ID_0_1_Cls.Parameters.Theta.Home:.3f})')

    # Set the absolute position of the robot joints.
    Mechanism_ID_0_1_Cls.Set_Absolute_Joint_Position(Mechanism_ID_0_1_Cls.Parameters.Theta.Home, CONST_T_0, CONST_T_1)

    # The last frame on which the animation stops.
    bpy.context.scene.frame_end = np.int32(CONST_T_1 * fps)

    # Get the absolute positions of the joints of the mechanism.
    print('[INFO] Absolute Joint Positions (actual):')
    print(f'[INFO] >> Joint_0({Mechanism_ID_0_1_Cls.Theta:.3f})')

    # Get the homogeneous transformation matrix of the mechanism end-effector (shuttle). Parameters position 
    # and orientation (euler angles).
    print('[INFO] Tool Center Point (TCP):')
    print(f'[INFO] >> p: x({Mechanism_ID_0_1_Cls.T_EE.p.x:.3f}), y({Mechanism_ID_0_1_Cls.T_EE.p.y:.3f}), z({Mechanism_ID_0_1_Cls.T_EE.p.z:.3f})')
    Euler_Angles = Mechanism_ID_0_1_Cls.T_EE.Get_Rotation('ZYX')
    print(f'[INFO] >> Euler Angles: x({Euler_Angles.x:.3f}), y({Euler_Angles.y:.3f}), z({Euler_Angles.z:.3f})')
if __name__ == '__main__':
    main()