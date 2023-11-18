# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Blender/Parameters/Camera
import RoLE.Blender.Parameters.Camera
#   ../RoLE/Blender/Utilities
import RoLE.Blender.Utilities
#   ../RoLE/Blender/Core
import RoLE.Blender.Core
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters

import RoLE.Transformation.Utilities.Mathematics as Mathematics

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
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = RoLE.Blender.Parameters.Camera.Right_View_Camera_Parameters_Str

def main():
    """
    Description:
        A program that demonstrates how to work with a specific class to control a robotic arm in Blender scene.
    """
    
    # Deselect all objects in the current scene.
    RoLE.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    RoLE.Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if RoLE.Blender.Utilities.Object_Exist('Camera'):
        RoLE.Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)
    
    # Initialization of the class to work with a robotic arm object in a Blender scene.
    Robot_ID_0_Cls = RoLE.Blender.Core.Robot_Cls(CONST_ROBOT_TYPE, {'Viewpoint_EE': False, 'Colliders': False, 
                                                                   'Workspace': False})
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    
    """
    # EPSON_LS3_B401S_Str
    th_0 = np.array([Mathematics.Degree_To_Radian(-40.0), Mathematics.Degree_To_Radian(50.0), 0.0, Mathematics.Degree_To_Radian(-30.0)],
                     dtype = np.float64)
    th_1 = np.array([Mathematics.Degree_To_Radian(115.0), Mathematics.Degree_To_Radian(-20.0), 0.10, Mathematics.Degree_To_Radian(15.0)],
                     dtype = np.float64)
    """
                                     
    # ...
    #Robot_ID_0_Cls.Reset('Individual', th_0)

    
if __name__ == '__main__':
    main()
