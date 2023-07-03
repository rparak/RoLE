# System (Default)
import sys
#   Add access if it is not in the system path.
sys.path.append('../../..')
# Custom Library:
#   ../Lib/Manipulator/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Manipulator/Parameters
import Lib.Parameters.Robot as Parameters

def main():
    """
    Description:
        Find the initialization (zero) configuration parameters for each joint of the robotic arm.

        Note:
            The configuration of each joint in which the robot arm is in zero absolute position.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = Parameters.ABB_IRB_120_Str

    """
    Description:
        Find the zero configuration of the homogeneous matrix of each joint using the modified 
        forward kinematics calculation method.
    """
    Robot_Str.T.Zero_Cfg = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

    for i, T_i in enumerate(Robot_Str.T.Zero_Cfg):
        # Get the translational and rotational part from the transformation matrix.
        p = T_i.p; Euler_Angles = T_i.Get_Rotation('ZYX'); Quaternions = T_i.Get_Rotation('QUATERNION')

        # Zero configuration of the homogeneous matrix in the current episode.
        #   Joint_{i}: p, R (Euler Angles and Quaternions)
        print(f'[INFO] Homogeneous matrix T_{i} in iteration {i}:')
        print(f'[INFO] >> p: [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]')
        print(f'[INFO] >> Euler Angles: [{Euler_Angles.x:.3f}, {Euler_Angles.y:.3f}, {Euler_Angles.z:.3f}]')
        print(f'[INFO] >> Quaternions: [{Quaternions.w:.5f}, {Quaternions.x:.5f}, {Quaternions.y:.5f}, {Quaternions.z:.5f}]')
    
if __name__ == '__main__':
    main()
