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
#       ../Blender/Configuration/Parameters
import Blender.Configuration.Parameters
# Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
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
# If the value is 'True', the homogeneous transformation matrix of the robot 
# base will be obtained from the Blender environment.
CONST_USE_BLENDER_ROBOT_BASE = False
# Animation stop(t_0), start(t_1) time in seconds.
CONST_T_0 = 0.0
CONST_T_1 = 2.0

def main():
    """
    Description:
        A program to calculate the inverse kinematics (IK) of the RRPR robotic structure (called SCARA) using an analytical method.

        Two methods can be used to obtain IK solutions: 'All' or 'Best'.
            1\ 'All': Obtain the all possible solutions.
            2\ 'Best': Automatically obtain the best solution.
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

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Blender.Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # Obtain the desired homogeneous transformation matrix T of the tool center point (TCP).
    TCP_Position = RoLE.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]

    # Modification of the robot base.
    if CONST_USE_BLENDER_ROBOT_BASE == True:
        Robot_Str.T.Base = HTM_Cls(bpy.data.objects[f'{Robot_Str.Name}_ID_{Robot_Str.Id:03}'].matrix_basis, 
                                np.float64)  

    # Initialization of the class to work with a robotic arm object in a Blender scene.
    Robot_ID_0_Cls = Blender.Robot.Core.Robot_Cls(Robot_Str, CONST_PROPERTIES)
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    # Reset the absolute position of the robot joints to the intial position.
    Robot_ID_0_Cls.Reset('Individual', abs_j_pos_0)
    
    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Analytical(TCP_Position, Robot_ID_0_Cls.Theta, Robot_ID_0_Cls.Parameters, 'Best')
    
    # Get the FPS (Frames Per Seconds) value from the Blender settings.
    fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base

    # The first frame on which the animation starts.
    bpy.context.scene.frame_start = np.int32(CONST_T_0 * fps)

    # Set the absolute position of the robot joints.
    Robot_ID_0_Cls.Set_Absolute_Joint_Position(theta, CONST_T_0, CONST_T_1)

    # The last frame on which the animation stops.
    bpy.context.scene.frame_end = np.int32(CONST_T_1 * fps)

    # Display results.
    print(f'[INFO] Absolute Joint Positions:')
    print(f'[INFO] >> position_err = {info["error"]["position"]}, orientation_err = {info["error"]["orientation"]}')
    print(f'[INFO] >> theta = {theta}')
    print(f'[INFO] >> is_close_singularity = {info["is_close_singularity"]}')
    print(f'[INFO] >> is_self_collision = {info["is_self_collision"]}')

    # Check that the calculation has been performed successfully.
    accuracy = info['error']['position'] + info['error']['orientation']
    if accuracy <= 1e-5:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')
        
if __name__ == '__main__':
    main()
