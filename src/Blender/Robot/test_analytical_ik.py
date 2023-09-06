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
#   ../Lib/Transformation/Core
import Lib.Transformation.Core as Transformation
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core


"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Lib.Blender.Parameters.Camera.Right_View_Camera_Parameters_Str

def main():
    """
    Description:
        ...
    """
    
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if Lib.Blender.Utilities.Object_Exist('Camera'):
        Lib.Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)
    
    # Set the structure of the main parameters of the controlled robot.
    Robot_ID_0_Cls = Lib.Blender.Core.Robot_Cls(CONST_ROBOT_TYPE, {'Viewpoint_EE': False, 'Colliders': False, 
                                                                   'Workspace': False})
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    # Reset the absolute position of the robot joints to the 'Home'.
    Robot_ID_0_Cls.Reset('Home')
    
    # Get the the absolute positions of the robot's joints.
    print('[INFO] Absolute Joint Positions (actual):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Theta):
        print(f'[INFO] >> Joint_{i}({th_i + 0.0:.3f})')


    # ...
    #Lib.Blender.Utilities.Set_Object_Transformation('TCP_Position_Viewpoint', Robot_ID_0_Cls.T_EE)

    # ...
    TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects['TCP_Position_Viewpoint'].matrix_basis, 
                                                                        np.float32)
    # ..
    theta = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, Robot_ID_0_Cls.Theta, Robot_ID_0_Cls.Parameters, 'All')
    
    # Reset the absolute position of the robot joints to the 'Individual'.
    #Robot_ID_0_Cls.Reset('Individual', theta[1])
    
    print(theta[0])
if __name__ == '__main__':
    main()