import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Custom Script:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

"""
Notes:
    1) Set the origin (translation position) of individual objects.
        Note:
            a) Right Click -> Snap       -> Cursor to Selected
            b) View        -> 3D Cursor  -> Location
            c) Right Click -> Set Origin -> Origin to 3D Cursor

    2) Reset the orientation of individual objects.
        Note: 
            a) (CTRL + a) -> Rotation
            b) The orientation of the object is now in the zero position.
            c) If I need the object to be oriented e.g. to 90 deg. (1.57.. rad.) in the X axis, I simply set 
               the X orientation to the opposite value of the desired orientation (-90 deg.).
            d) (CTRL + a) -> Rotation
            e) Set the object to the desired orientation.
"""

def main():
    """
    Description:
        Find the initialization (zero) configuration parameters for each joint of the robotic arm.

        Note:
            The configuration of each joint in which the robot arm is in zero absolute position.
    """

    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
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

    # Function to hide and unhide the visibility of objects.
    Lib.Blender.Utilities.Object_Visibility('Viewpoint', True)
    
    # Add viewpoints with the correct transformation to object joints.
    Lib.Blender.Utilities.Add_Viewpoints('Viewpoint', Robot_Str.T.Zero_Cfg)
    
if __name__ == '__main__':
    main()
