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

# Initialization of the absolute joint position of the robot structure.
#   Type: ABB IRB 120
CONST_ABB_IRB_120_ZERO_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64))
CONST_ABB_IRB_120_HOME_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 90.0, 0.0], dtype=np.float64))
CONST_ABB_IRB_120_TEST_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([20.0, -20.0, 20.0, -20.0, 20.0, -20.0], dtype=np.float64))
#   Type: Universal Robots UR3
CONST_UR3_ZERO_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64))
CONST_UR3_HOME_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([-90.0, -90.0, 0.0, -90.0, 0.0, 0.0], dtype=np.float64))
CONST_UR3_TEST_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([20.0, -20.0, 20.0, -20.0, 20.0, -20.0], dtype=np.float64))
#   Type: ABB IRB 14000 (Right)
CONST_ABB_IRB_14000_R_ZERO_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64))
CONST_ABB_IRB_14000_R_HOME_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, -130.0, -135.0, 30.0, 0.0, 40.0, 0.0], dtype=np.float64))
CONST_ABB_IRB_14000_R_TEST_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([20.0, -20.0, 20.0, -20.0, 20.0, -20.0, 20.0], dtype=np.float64))
#   Type: ABB IRB 14000 (Left)
CONST_ABB_IRB_14000_L_ZERO_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64))
CONST_ABB_IRB_14000_L_HOME_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, -130.0,  135.0, 30.0, 0.0, 40.0, 0.0], dtype=np.float64))
CONST_ABB_IRB_14000_L_TEST_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([20.0, -20.0, 20.0, -20.0, 20.0, -20.0, 20.0], dtype=np.float64))
#   Type: Epson LS3-B401S
CONST_EPSON_LS3_B401S_ZERO_JOINT_ORIENTATION = [Mathematics.Degree_To_Radian(0.0), Mathematics.Degree_To_Radian(0.0), 0.0, Mathematics.Degree_To_Radian(0.0)]
CONST_EPSON_LS3_B401S_HOME_JOINT_ORIENTATION = [Mathematics.Degree_To_Radian(90.0), Mathematics.Degree_To_Radian(0.0), 0.0, Mathematics.Degree_To_Radian(0.0)]
CONST_EPSON_LS3_B401S_TEST_JOINT_ORIENTATION = [Mathematics.Degree_To_Radian(-20.0), Mathematics.Degree_To_Radian(20.0), -0.05, Mathematics.Degree_To_Radian(50.0)]

# Parameter for the desired absolute positions of the robot arm joints.
CONST_DESIRED_JOINT_ORIENTATION = CONST_UR3_ZERO_JOINT_ORIENTATION

def main():
    """
    Description:
        ...
    """
    
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()
    
    # ...
    Robot_ID_0_Cls = Lib.Blender.Core.Robot_Cls(CONST_ROBOT_TYPE, True)
    print(f'[INFO] Robot Name: {Robot_ID_0_Cls.Parameters.Name}')

    print('[INFO] Absolute Joint Positions (desired):')
    for i, th_i in enumerate(CONST_DESIRED_JOINT_ORIENTATION):
        print(f'[INFO] >> Joint_{i}({Mathematics.Radian_To_Degree(th_i):.3f})')

    # Set the absolute position of the robot joints.
    Robot_ID_0_Cls.Set_Absolute_Joint_Position(CONST_DESIRED_JOINT_ORIENTATION)

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