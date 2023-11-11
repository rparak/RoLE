import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: Industrial Robotics Library for Everyone (IRLE)
#   ../IRLE/Blender/Utilities
import IRLE.Blender.Utilities
#   ../IRLE/Kinematics/Core
import IRLE.Kinematics.Core
#   ../IRLE/Parameters/Robot
import IRLE.Parameters.Robot as Parameters
#   ../IRLE/Blender/Parameters/Camera
import IRLE.Blender.Parameters.Camera

"""
Description:
    Open Viewpoint.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/Open_Industrial_Robotics/Blender/Helpers
        $ blender Viewpoint.blend
"""

"""
Notes:
    1) Set the origin (translation position) of individual objects.
        Note:
            a) Right Click -> Snap       -> Cursor to Selected
            b) View        -> 3D Cursor  -> Location
            c) Right Click -> Set Origin -> Origin to 3D Cursor

    2) Reset the orientation of individual objects.
        Note: 
            a) (CTRL + a) -> Rotation
            b) The orientation of the object is now in the zero position.
            c) If I need the object to be oriented e.g. to 90 deg. (1.57.. rad.) in the X axis, I simply set 
               the X orientation to the opposite value of the desired orientation (-90 deg.).
            d) (CTRL + a) -> Rotation
            e) Set the object to the desired orientation.
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_L_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = IRLE.Blender.Parameters.Camera.Right_View_Camera_Parameters_Str

def main():
    """
    Description:
        Find the initialization (zero) configuration parameters for each joint of the robotic arm.

        Note:
            The configuration of each joint in which the robot arm is in zero absolute position.
    """

    # Deselect all objects in the current scene.
    IRLE.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    IRLE.Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if IRLE.Blender.Utilities.Object_Exist('Camera'):
        IRLE.Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)

    # Removes joint viewpoints if they exist in the current scene.
    i = 0
    while True:
        if IRLE.Blender.Utilities.Object_Exist(f'Viewpoint_Joint_{i}') == True:
            IRLE.Blender.Utilities.Remove_Object(f'Viewpoint_Joint_{i}')
        else:
            break     
        i += 1

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE
    
    """
    Description:
        Find the zero configuration of the homogeneous transformation matrix of each joint using the modified 
        forward kinematics calculation method.
    """
    Robot_Str.T.Zero_Cfg = IRLE.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

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

    # Function to hide and unhide the visibility of objects.
    IRLE.Blender.Utilities.Object_Visibility('Viewpoint', True)
    
    # Add viewpoints with the correct transformation to object joints.
    IRLE.Blender.Utilities.Add_Viewpoints('Viewpoint', Robot_Str.T.Zero_Cfg)
    
if __name__ == '__main__':
    main()
