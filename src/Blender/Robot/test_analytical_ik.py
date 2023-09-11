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
    Open EPSON_LS3_B401S.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/Open_Industrial_Robotics/Blender/Robot
        $ blender EPSON_LS3_B401S.blend
"""

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
        A program to calculate the inverse kinematics (IK) of the RRPR robotic structure (called SCARA) using an analytical method.

        Two methods can be used to obtain IK solutions: 'All' or 'Best'.
            1\ 'All': Obtain the all possible solutions.
            2\ 'Best': Automatically obtain the best solution.

        Note:
            The position and orientation of the 'Viewpoint' object, which is the input to the inverse kinematics 
            function, is set by the user.
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

    # Obtain the homogeneous transformation matrix of the 'Viewpoint' object.
    TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects['TCP_Position_Viewpoint'].matrix_basis, 
                                                                        np.float32)
    
    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (error, theta) = Lib.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, Robot_ID_0_Cls.Theta, Robot_ID_0_Cls.Parameters, 'Best')
    
    # Display results.
    print(f'[INFO] Absolute Joint Positions:')
    print(f'[INFO] >> position_err = {error["position"]}, orientation_err = {error["orientation"]}')
    print(f'[INFO] >> theta = {theta}')

    # Reset the absolute position of the robot joints to the 'Individual'.
    Robot_ID_0_Cls.Reset('Individual', theta)
        
if __name__ == '__main__':
    main()