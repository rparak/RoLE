# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Custom Script:
#   ../Lib/Blender/Parameters/Camera
import Lib.Blender.Parameters.Camera
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Blender/Core
import Lib.Blender.Core
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics


"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_L_Str
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

    CONST_ROBOT_TYPE.T.Base = Transformation.Homogeneous_Transformation_Matrix_Cls([[ 0.5716,  0.1048, 0.8138, 0.0536],
                 [-0.6170,  0.7088, 0.3420, 0.0725],
                 [-0.5410, -0.6976, 0.4698, 0.4149],
                 [    0.0,     0.0,    0.0,    1.0]], np.float32)

    # Set the camera (object) transformation and projection.
    if Lib.Blender.Utilities.Object_Exist('Camera'):
        Lib.Blender.Utilities.Set_Camera_Properties('Camera', CONST_CAMERA_TYPE)
    
    # Set the structure of the main parameters of the controlled robot.
    Robot_ID_0_Cls = Lib.Blender.Core.Robot_Cls(CONST_ROBOT_TYPE, True)
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}_ID_{Robot_ID_0_Cls.Parameters.Id:03}')

    # Reset the absolute position of the robot joints to the 'Zero'.
    Robot_ID_0_Cls.Reset('Zero')
    
    print('[INFO] Absolute Joint Positions (desired):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Parameters.Theta.Home):
        print(f'[INFO] >> Joint_{i}({th_i:.3f})')

    # Set the absolute position of the robot joints.
    Robot_ID_0_Cls.Set_Absolute_Joint_Position(Robot_ID_0_Cls.Parameters.Theta.Home)

    # Get the the absolute positions of the robot's joints.
    print('[INFO] Absolute Joint Positions (actual):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Theta):
        print(f'[INFO] >> Joint_{i}({th_i:.3f})')

    # Get the homogeneous transformation matrix of the robot end-effector. Parameters position 
    # and orientation (euler angles).
    print('[INFO] Tool Center Point (TCP):')
    print(f'[INFO] >> p: x({Robot_ID_0_Cls.T_EE.p.x:.3f}), y({Robot_ID_0_Cls.T_EE.p.y:.3f}), z({Robot_ID_0_Cls.T_EE.p.z:.3f})')
    Euler_Angles = Robot_ID_0_Cls.T_EE.Get_Rotation('ZYX')
    print(f'[INFO] >> Euler Angles: x({Euler_Angles.x:.3f}), y({Euler_Angles.y:.3f}), z({Euler_Angles.z:.3f})')
    
if __name__ == '__main__':
    main()