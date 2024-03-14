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
#       ../Blender/Parameters/Camera
import Blender.Parameters.Camera
#       ../Blender/Utilities
import Blender.Utilities
#       ../Blender/Mechanism/Core
import Blender.Mechanism.Core
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Mechanism
import RoLE.Parameters.Mechanism as Parameters
#       ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation

"""
Description:
    Open {mechanism_name}.blend from the Blender folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/RoLE/Blender/Mechanism
        $ blender {mechanism_name}.blend

    Note:
        Where the variable 'mechanism_name' is the name of the controlled mechanism to be used.
"""

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled mechanism.
CONST_MECHANISM_TYPE = Parameters.SMC_LEFB25_14000_0_1_Str
# Set the structure of the main parameters of the camera.
CONST_CAMERA_TYPE = Blender.Parameters.Camera.Right_View_Camera_Parameters_Str
# The properties of the mechanism structure in the Blender environment.
CONST_PROPERTIES = {'fps': 100, 'visibility': {'Viewpoint_EE': False, 'Colliders': False, 
                                               'Ghost': True}}
# Animation stop(t_0), start(t_1) time in seconds.
CONST_T_0 = 0.0
CONST_T_1 = 2.0

def main():
    """
    Description:
        A program to calculate the solution of the inverse kinematics (IK) of the mechanism structure.

        Note:
            The position of the 'Viewpoint' object, which is the input to the inverse kinematics 
            function, is set by the user.

            If the 'Viewpoint' object is not part of the environment, copy it from the following Blender file:
                ../Blender/Helpers/Viewpoint.blend

            and change the name from 'Viewpoint' to 'TCP_SMC_LEFB25_14000_ID_001/002'.
    """
    
    # Deselect all objects in the current scene.
    Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Blender.Utilities.Remove_Animation_Data()

    # Set the camera (object) transformation and projection.
    if Blender.Utilities.Object_Exist('Camera'):
        Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)

    # Initialization of the class to work with a mechanism object in a Blender scene.
    Mechanism_ID_0_1_Cls = Blender.Mechanism.Core.Mechanism_Cls(CONST_MECHANISM_TYPE, CONST_PROPERTIES)
    print(f'[INFO] Mechanism Name: {Mechanism_ID_0_1_Cls.Parameters.Name}_ID_{Mechanism_ID_0_1_Cls.Parameters.Id:03}')
    
    # Reset the absolute position of the mechanism joints to the 'Zero'.
    Mechanism_ID_0_1_Cls.Reset('Zero')
    
    # Obtain the homogeneous transformation matrix of the 'Viewpoint' object.
    T = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects[f'TCP_{Mechanism_ID_0_1_Cls.Parameters.Name}_ID_{Mechanism_ID_0_1_Cls.Parameters.Id:03}'].matrix_basis, 
                                                             np.float64)
    
    # Obtain the inverse kinematics (IK) solution of the mechanism structure.
    (info, theta) = Mechanism_ID_0_1_Cls.Get_Inverse_Kinematics_Solution(T, True)
    
    if info == True:
        # Get the FPS (Frames Per Seconds) value from the Blender settings.
        fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base

        # The first frame on which the animation starts.
        bpy.context.scene.frame_start = np.int32(CONST_T_0 * fps)

        print('[INFO] Absolute Joint Positions (desired):')
        print(f'[INFO] >> Joint_0({theta:.3f})')

        # Set the absolute position of the mechanism joints.
        Mechanism_ID_0_1_Cls.Set_Absolute_Joint_Position(theta, CONST_T_0, CONST_T_1)

        # The last frame on which the animation stops.
        bpy.context.scene.frame_end = np.int32(CONST_T_1 * fps)

        # Get the absolute positions of the joints of the mechanism.
        print('[INFO] Absolute Joint Positions (actual):')
        print(f'[INFO] >> Joint_0({Mechanism_ID_0_1_Cls.Theta + 0.0:.3f})')

        # Get the homogeneous transformation matrix of the mechanism end-effector (shuttle). Parameters position 
        # and orientation (euler angles).
        print('[INFO] Tool Center Point (TCP):')
        print(f'[INFO] >> p: x({Mechanism_ID_0_1_Cls.T_EE.p.x + 0.0:.3f}), y({Mechanism_ID_0_1_Cls.T_EE.p.y + 0.0:.3f}), z({Mechanism_ID_0_1_Cls.T_EE.p.z + 0.0:.3f})')
        Euler_Angles = Mechanism_ID_0_1_Cls.T_EE.Get_Rotation('ZYX') + [0.0, 0.0, 0.0]
        print(f'[INFO] >> Euler Angles: x({Euler_Angles.x:.3f}), y({Euler_Angles.y:.3f}), z({Euler_Angles.z:.3f})')
    else:
        print('[WARNING] There is an issue during the execution of the TCP (tool center point) target.')
        print(f'[WARNING] >> p = {T.p.all()}')
        print(f'[WARNING] >> Quaternion = {T.Get_Rotation("QUATERNION").all()}')
    
if __name__ == '__main__':
    main()
