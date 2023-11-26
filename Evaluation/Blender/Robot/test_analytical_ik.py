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
#       ../RoLE/Blender/Core
import Blender.Core
# Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core

"""
Description:
    Open EPSON_LS3_B401S.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/RoLE/Blender/Robot
        $ blender EPSON_LS3_B401S.blend
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Blender.Parameters.Camera.Right_View_Camera_Parameters_Str
# The properties of the robot structure in the Blender environment.
CONST_PROPERTIES = {'fps': 100, 'visibility': {'Viewpoint_EE': False, 'Colliders': False, 
                                               'Workspace': False, 'Ghost': False}}

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

            If the 'Viewpoint' object is not part of the environment, copy it from the following Blender file:
                ../Blender/Helpers/Viewpoint.blend
    """
    
    # Deselect all objects in the current scene.
    Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if Blender.Utilities.Object_Exist('Camera'):
        Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)
    
    # Initialization of the class to work with a robotic arm object in a Blender scene.
    Robot_ID_0_Cls = Blender.Core.Robot_Cls(CONST_ROBOT_TYPE, {'Viewpoint_EE': False, 'Colliders': False, 
                                                                   'Workspace': False})
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    # Reset the absolute position of the robot joints to the 'Home'.
    Robot_ID_0_Cls.Reset('Home')

    # Obtain the homogeneous transformation matrix of the 'Viewpoint' object.
    TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects[f'TCP_{Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}'].matrix_basis, 
                                                                        np.float64)
    
    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, Robot_ID_0_Cls.Theta, Robot_ID_0_Cls.Parameters, 'Best')
    
    # Display results.
    print(f'[INFO] Absolute Joint Positions:')
    print(f'[INFO] >> position_err = {info["error"]["position"]}, orientation_err = {info["error"]["orientation"]}')
    print(f'[INFO] >> theta = {theta}')

    # Reset the absolute position of the robot joints to the 'Individual'.
    Robot_ID_0_Cls.Reset('Individual', theta)
        
if __name__ == '__main__':
    main()
