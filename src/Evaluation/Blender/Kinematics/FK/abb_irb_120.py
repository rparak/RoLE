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
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

"""
Description: 
    Initialization of an absolute joint position for each target.
        
    ABB IRB 120:
        Joint 1: [+/- 165] [°]
        Joint 2: [+/- 110] [°]
        Joint 3: [-110, +70] [°]
        Joint 4: [+/- 160] [°]
        Joint 5: [+/- 120] [°]
        Joint 6: [+/- 400] [°]
"""
CONST_ZERO_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64))
CONST_HOME_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 90.0, 0.0], dtype=np.float64))
CONST_TEST_JOINT_ORIENTATION = Mathematics.Degree_To_Radian(np.array([20.0, -20.0, 20.0, -20.0, 20.0, -20.0], dtype=np.float64))

# Parameter for the desired absolute positions of the robot arm joints.
CONST_DESIRED_JOINT_ORIENTATION = CONST_ZERO_JOINT_ORIENTATION

def main():
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()
    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()
    
    # The structure of the main parameters of the robot.
    Parameters.ABB_IRB_120_Str.T.Base = HTM_Cls(bpy.data.objects[Parameters.ABB_IRB_120_Str.Name].matrix_basis, np.float32)

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = Parameters.ABB_IRB_120_Str

    # Function to hide and unhide the visibility of objects.
    Lib.Blender.Utilities.Object_Visibility('Viewpoint', True)
    
    """
    Description:
        Find the zero configuration of the homogeneous matrix of each joint using forward kinematics. 
    """
    Robot_Str.T.Zero_Cfg = Kinematics.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]
    
    # Set the absolute position of the robot joints.
    Lib.Blender.Utilities.Set_Absolute_Joint_Position(CONST_DESIRED_JOINT_ORIENTATION, Robot_Str)
    
    # Get the the absolute positions of the robot's joints.
    print('[INFO] Absolute positions of the robot joints:')
    for i, th_i in enumerate(Lib.Blender.Utilities.Get_Absolute_Joint_Position(Robot_Str)):
        print(f'[INFO] Joint {i}: {Mathematics.Radian_To_Degree(th_i):.3f}')
    
    """
    Description:
        Add viewpoint to the end-effector (tool) of the robotic arm.
    """
    Lib.Blender.Utilities.Set_Object_Transformation('Viewpoint', Kinematics.Forward_Kinematics(CONST_DESIRED_JOINT_ORIENTATION, 'Modified', Robot_Str)[1])
    
if __name__ == '__main__':
    main()