# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Blender/Core
import Lib.Blender.Core
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics


def main():
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()
    
    ABB_IRB_120_Cls = Lib.Blender.Core.Robot_Cls(Parameters.ABB_IRB_120_Str)

    # Get the the absolute positions of the robot's joints.
    print('[INFO] Absolute positions of the robot joints:')
    for i, th_i in enumerate(ABB_IRB_120_Cls.Theta):
        print(f'[INFO] Joint {i}: {Mathematics.Radian_To_Degree(th_i):.3f}')
    
if __name__ == '__main__':
    main()