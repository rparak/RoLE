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
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

def main():
    """
    Description:
        ...
    """
    
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()
    
    # Set the structure of the main parameters of the controlled robot.
    Robot_ID_0_Cls = Lib.Blender.Core.Robot_Cls(CONST_ROBOT_TYPE, True)
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}')

    # Reset the absolute position of the robot joints to the 'Zero'.
    Robot_ID_0_Cls.Reset('Zero')
    
    print('[INFO] Absolute Joint Positions (desired):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Parameters.Theta.Home):
        print(f'[INFO] >> Joint_{i}({Mathematics.Radian_To_Degree(th_i):.3f})')

    # Set the absolute position of the robot joints.
    Robot_ID_0_Cls.Set_Absolute_Joint_Position(Robot_ID_0_Cls.Parameters.Theta.Home)

    # Get the the absolute positions of the robot's joints.
    print('[INFO] Absolute Joint Positions (actual):')
    for i, th_i in enumerate(Robot_ID_0_Cls.Theta):
        print(f'[INFO] >> Joint_{i}({Mathematics.Radian_To_Degree(th_i):.3f})')

    # Get the homogeneous transformation matrix of the robot end-effector. Parameters position 
    # and orientation (euler angles).
    print('[INFO] Tool Center Point (TCP):')
    print(f'[INFO] >> p: x({Robot_ID_0_Cls.T_EE.p.x:.3f}), y({Robot_ID_0_Cls.T_EE.p.y:.3f}), z({Robot_ID_0_Cls.T_EE.p.z:.3f})')
    Euler_Angles = Robot_ID_0_Cls.T_EE.Get_Rotation('ZYX')
    print(f'[INFO] >> Euler Angles: x({Euler_Angles.x:.3f}), y({Euler_Angles.y:.3f}), z({Euler_Angles.z:.3f})')
    
if __name__ == '__main__':
    main()