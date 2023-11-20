# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' +  'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

def main():
    """
    Description:
        Find the initialization (zero) configuration parameters for each joint of the robotic arm.

        Note:
            The configuration of each joint in which the robot arm is in zero absolute position.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    """
    Description:
        Find the zero configuration of the homogeneous matrix of each joint using the modified 
        forward kinematics calculation method.
    """
    Robot_Str.T.Zero_Cfg = RoLE.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

    for i, T_i in enumerate(Robot_Str.T.Zero_Cfg):
        # Get the translational and rotational part from the transformation matrix.
        p = np.round(T_i.p.all(), 5) + [0.0, 0.0, 0.0]; Euler_Angles = np.round(T_i.Get_Rotation('ZYX').all(), 5) + [0.0, 0.0, 0.0]
        Quaternions = np.round(T_i.Get_Rotation('QUATERNION').all(), 5) + [0.0, 0.0, 0.0, 0.0]

        # Zero configuration of the homogeneous matrix in the current episode.
        #   Joint_{i}: p, R (Euler Angles and Quaternions)
        print(f'[INFO] Homogeneous matrix T_{i} in iteration {i}:')
        print(f'[INFO] >> p: [{p[0]:.05f}, {p[1]:.05f}, {p[2]:.05f}]')
        print(f'[INFO] >> Euler Angles: [{Euler_Angles[0]:.05f}, {Euler_Angles[1]:.05f}, {Euler_Angles[2]:.05f}]')
        print(f'[INFO] >> Quaternions: [{Quaternions[0]:.05f}, {Quaternions[1]:.05f}, {Quaternions[2]:.05f}, {Quaternions[3]:.05f}]')
    
if __name__ == '__main__':
    main()
