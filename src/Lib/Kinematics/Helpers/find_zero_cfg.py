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
        p = T_i.p.all(); R_ea = T_i.Get_Rotation('ZYX').all()

        # Zero configuration of the homogeneous matrix in the current episode.
        #   Joint_{i}: p, R (Euler Angles or Quaternions)
        print(f'[INFO] Homogeneous matrix T_{i} in iteration {i}:')
        print(f'[INFO] Translation part (X, Y, Z [metres]): |X = {p[0]:.5f} | Y = {p[1]:.5f} | Z = {p[2]:.5f}|')
        print(f'[INFO] Rotation part (Euler Angles [rad]):  |X = {R_ea[0]:.5f} | Y = {R_ea[1]:.5f} | Z = {R_ea[2]:.5f}|')       
    
if __name__ == '__main__':
    main()
