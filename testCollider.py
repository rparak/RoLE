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
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters

"""
Description:
    Open {robot_name}.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/Industrial_Robots_Kinematics/Blender/Robot
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
# Animation stop(t_0), start(t_1) time in seconds.
CONST_T_0 = 0.0
CONST_T_1 = 5.0

def main():
    """
    Description:
        A program that demonstrates how to work with a specific class to control a robotic arm in Blender scene.
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
    
    """
    Description:
        Find the zero configuration of the homogeneous matrix of each joint using the modified 
        forward kinematics calculation method.
    """
    Robot_Str.T.Zero_Cfg = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]
    
    """
    #print(Robot_ID_0_Cls.T_EE.p.all())
    Lib.Blender.Utilities.Set_Object_Transformation('Viewpoint_EE_ABB_IRB_14000_R_ID_001', Robot_ID_0_Cls.T_EE)
    """
    
    import Lib.Kinematics.Core as Kinematics

    Robot_Str.T.Zero_Cfg = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

    for i, T_i in enumerate(Robot_Str.T.Zero_Cfg):
        # Get the translational and rotational part from the transformation matrix.
        p = np.round(T_i.p.all(), 5) + [0.0, 0.0, 0.0]; Euler_Angles = np.round(T_i.Get_Rotation('ZYX').all(), 5) + [0.0, 0.0, 0.0]
        Quaternions = np.round(T_i.Get_Rotation('QUATERNION').all(), 5) + [0.0, 0.0, 0.0, 0.0]

        # Zero configuration of the homogeneous matrix in the current episode.
        #   Joint_{i}: p, R (Euler Angles and Quaternions)
        print(f'[INFO] Homogeneous matrix T_{i} in iteration {i}:')
        print(f'[INFO] >> p: [{p[0]:.05f}, {p[1]:.05f}, {p[2]:.05f}]')
        print(f'[INFO] >> Euler Angles: [{Euler_Angles[0]:.05f}, {Euler_Angles[1]:.05f}, {Euler_Angles[2]:.05f}]')
        print(f'[INFO] >> Quaternions: [{Quaternions[0]:.05f}, {Quaternions[1]:.05f}, {Quaternions[2]:.05f}, {Quaternions[3]:.05f}]')
        
    Lib.Blender.Utilities.Set_Object_Transformation(f'Joint_6_Collider', Robot_Str.T.Zero_Cfg[-1])
    
if __name__ == '__main__':
    main()