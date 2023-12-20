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
# Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core

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
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Blender.Parameters.Camera.Right_View_Camera_Parameters_Str
# The properties of the robot structure in the Blender environment.
CONST_PROPERTIES = {'fps': 100, 'visibility': {'Viewpoint_EE': False, 'Colliders': False, 
                                               'Workspace': False, 'Ghost': False}}
# Numerical IK Parameters.
#   Name of the numerical method to be used to calculate the IK solution.
#       'Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton', 
#       'Levenberg-Marquardt'
CONST_NIK_METHOD = 'Levenberg-Marquardt'
#   The properties of the inverse kinematics solver.
#       'tolerance': 1e-03 -> 'Jacobian-Transpose'
#       'tolerance': 1e-30 -> 'Newton-Raphson', 'Gauss-Newton', and 'Levenberg-Marquardt'
CONST_IK_PROPERTIES = {'delta_time': 0.1, 'num_of_iteration': 500, 
                       'tolerance': 1e-30}

def main():
    """
    Description:
        A program to calculate the inverse kinematics (IK) of the individual robotic structure using a numerical method.

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
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialization of the class to work with a robotic arm object in a Blender scene.
    Robot_ID_0_Cls = Blender.Robot.Core.Robot_Cls(Robot_Str, CONST_PROPERTIES)
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    # Reset the absolute position of the robot joints to the 'Home'.
    Robot_ID_0_Cls.Reset('Home')

    # Obtain the homogeneous transformation matrix of the 'Viewpoint' object.
    TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects[f'TCP_{Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}'].matrix_basis, 
                                                                        np.float64)
    
    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(TCP_Position, Robot_ID_0_Cls.Theta, CONST_NIK_METHOD, Robot_ID_0_Cls.Parameters, 
                                                                      CONST_IK_PROPERTIES)
    
    # Reset the absolute position of the robot joints to the 'Individual'.
    Robot_ID_0_Cls.Reset('Individual', theta)

    # Display results.
    print(f'[INFO] Absolute Joint Positions:')
    print(f'[INFO] >> successful = {info["successful"]}')
    print(f'[INFO] >> iteration = {info["iteration"]}')
    print(f'[INFO] >> position_err = {info["error"]["position"]}, orientation_err = {info["error"]["orientation"]}')
    print(f'[INFO] >> theta = {theta}')
    print(f'[INFO] >> is_close_singularity = {info["is_close_singularity"]}')
    print(f'[INFO] >> is_self_collision = {info["is_self_collision"]}')

    # Check that the calculation has been performed successfully.
    accuracy = info["error"]["position"] + info["error"]["orientation"]
    if info["successful"] == True:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')
        
if __name__ == '__main__':
    main()
