# Numpy (Array computing) [pip3 install numpy]tp
import numpy as np
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Custom Library:
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

@dataclass
class DH_Parameters_Str:
    """
    Description:
        The auxiliary structure of the Denavit-Hartenberg (DH) parameters.

        Note 1:
            Private structure.

        Note 2:
            DH (Denavit-Hartenberg) parameters: 
    
            (1) theta_zero [Vector<float>]: Joint angle (Theta_i). Rotation part in radians.
                                            Unit: [radian]                        
            (2) a [Vector<float>]: Link length (a_i). Translation part in meters.
                                   Unit: [meter]
            (3) d [Vector<float>]: Link offset (d_i). Translation part in meters.
                                   Unit: [meter]
            (4) alpha [Vector<float>]: Link twist (alpha_i). Rotation part in radians.
                                       Unit: [radian]
    """

    # Standard Denavit-Hartenberg (DH):
    #       DH_theta_zero = th{i} + theta_zero{i}
    #       DH_a          = a{i}
    #       DH_d          = d{i}
    #       DH_alpha      = alpha{i}
    #   Unit [Matrix<float>]
    Standard: tp.List[tp.List[float]] = field(default_factory=list)
    # Modified Denavit-Hartenberg (DH):
    #       DH_theta_zero = th{i} + theta_zero{i}
    #       DH_a          = a{i - 1}
    #       DH_d          = d{i}
    #       DH_alpha      = alpha{i - 1}
    #   Unit [Matrix<float>]
    Modified: tp.List[tp.List[float]] = field(default_factory=list)

@dataclass
class Theta_Parameters_Str(object):
    """
    Description:
        The auxiliary structure of the joint (theta) parameters.

        Note:
            Private structure.
    """

    # Zero absolute position of each joint.
    #   Unit [Vector<float>]
    Zero: tp.List[float] = field(default_factory=list)
    # Home absolute position of each joint.
    #   Unit [Vector<float>]
    Home: tp.List[float] = field(default_factory=list)
    # Limits of absolute joint position in radians and meters.
    #   Unit [Matrix<float>]
    Limit: tp.List[tp.List[float]] = field(default_factory=list)
    # Parameters of the object (Blender robot arm).
    #   The name of the joints.
    #       Unit [Vector<string>]
    Name: tp.List[str] = field(default_factory=list)
    #   Identification of the type of joints.
    #       Note: R - Revolute, P - Prismatic
    #       Unit [Vector<string>]
    Type: tp.List[str] = field(default_factory=list)
    #   Identification of the axis of the absolute position of the joint. 
    #       Unit [Vector<string>]
    Axis: tp.List[str] = field(default_factory=list)
    #   Direction of the axis of the joint. 
    #       Note: +1 (Standard), -1 (Inverted)
    #       Unit [Vector<int>]
    Direction: tp.List[int] = field(default_factory=list)

@dataclass
class T_Parameters_Str:
    """
    Description:
        The auxiliary structure of the homogeneous transformation matrix {T} parameters.

        Note:
            Private structure.
    """

    # Homogeneous transformation matrix of the base.
    #   Unit [Matrix<float>]
    Base: tp.List[tp.List[float]] = field(default_factory=list)
    # Homogeneous transformation matrix of the end-effector (tool).
    #   Unit [Matrix<float>]
    End_Effector: tp.List[tp.List[float]] = field(default_factory=list)
    # The zero configuration of the homogeneous transformation 
    # matrix of each joint (theta). The method (Standard, Modified) chosen 
    # to determine the configuration depends on the specific task.
    #   Unit [Matrix<float>]
    Zero_Cfg: tp.List[tp.List[float]] = field(default_factory=list)

@dataclass
class Robot_Parameters_Str:
    """
    Description:
        The structure of the main parameters of the robot.

    Initialization of the Class (structure):
        Input:
            (1) name [string]: Name of the robotic structure.

    Example:
        Initialization:
            Cls = Robot_Parameters_Str(name)
            Cls.Name = ...
            ...
            Cls.T = ..
    """

    # Name of the robotic structure.
    #   Unit [string]
    Name: str = ''
    # Denavit-Hartenberg (DH) parameters.
    #   Unit [__DH_Parameters_Str(object)]
    DH: DH_Parameters_Str = field(default_factory=DH_Parameters_Str)
    # Absolute joint position (theta) parameters
    #   Unit [__Theta_Parameters_Str(object)]
    Theta: Theta_Parameters_Str = field(default_factory=Theta_Parameters_Str)
    # Homogeneous transformation matrix (T) parameters.
    #   Unit [__T_Parameters_Str(object)]
    T: T_Parameters_Str = field(default_factory=T_Parameters_Str)

"""
Robot Type - Universal Robots UR3:
    Absolute Joint Position:
        Joint 1 - 6: [+/- 360.0] [°] -> modified [+/- 180.0] [°]  

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [   0.0,      0.0,      0.0,     0.0,     0.0,    0.0]
        a          = [   0.0, -0.24365, -0.21325,     0.0,     0.0,    0.0]
        d          = [0.1519,      0.0,      0.0, 0.11235, 0.08535, 0.0819]
        alpha      = [  1.57,      0.0,      0.0,    1.57,   -1.57,    0.0]
"""
Universal_Robots_UR3_Str = Robot_Parameters_Str(Name = 'Universal_Robots_UR3_ID_001')
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
#   2\ Stand:
#       [[1.0, 0.0, 0.0,  0.0],
#        [0.0, 1.0, 0.0,  0.0],
#        [0.0, 0.0, 1.0, 0.02],
#        [0.0, 0.0, 0.0,  1.0]]
Universal_Robots_UR3_Str.T.Base = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                           [0.0, 1.0, 0.0, 0.0],
                                           [0.0, 0.0, 1.0, 0.0],
                                           [0.0, 0.0, 0.0, 1.0]], np.float32)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
Universal_Robots_UR3_Str.T.End_Effector = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                                   [0.0, 1.0, 0.0, 0.0],
                                                   [0.0, 0.0, 1.0, 0.0],
                                                   [0.0, 0.0, 0.0, 1.0]], np.float32)
# Denavit-Hartenberg (DH)
Universal_Robots_UR3_Str.DH.Standard = np.array([[0.0,      0.0,  0.1519,   1.5707963267948966],
                                                 [0.0, -0.24365,     0.0,                  0.0],
                                                 [0.0, -0.21325,     0.0,                  0.0],
                                                 [0.0,      0.0, 0.11235,   1.5707963267948966],
                                                 [0.0,      0.0, 0.08535,  -1.5707963267948966],
                                                 [0.0,      0.0,  0.0819,                  0.0]], dtype=np.float32) 
Universal_Robots_UR3_Str.DH.Modified = np.array([[0.0,      0.0,  0.1519,                  0.0],
                                                 [0.0,      0.0,     0.0,   1.5707963267948966],
                                                 [0.0, -0.24365,     0.0,                  0.0],
                                                 [0.0, -0.21325, 0.11235,                  0.0],
                                                 [0.0,      0.0, 0.08535,   1.5707963267948966],
                                                 [0.0,      0.0,  0.0819,  -1.5707963267948966]], dtype=np.float32)
# Zero/Home absolute position of each joint.
Universal_Robots_UR3_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float32)
Universal_Robots_UR3_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([-90.0, -90.0, 0.0, -90.0, 0.0, 0.0], 
                                                                            dtype=np.float64))
# Limits of absolute joint position.
Universal_Robots_UR3_Str.Theta.Limit = np.array([[-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793]], dtype = np.float32)
# Other parameters of the robot structure.
Universal_Robots_UR3_Str.Theta.Name = [f'Joint_1_{Universal_Robots_UR3_Str.Name}', f'Joint_2_{Universal_Robots_UR3_Str.Name}', 
                                       f'Joint_3_{Universal_Robots_UR3_Str.Name}', f'Joint_4_{Universal_Robots_UR3_Str.Name}', 
                                       f'Joint_5_{Universal_Robots_UR3_Str.Name}', f'Joint_6_{Universal_Robots_UR3_Str.Name}']
Universal_Robots_UR3_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R']
Universal_Robots_UR3_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z']

"""
Robot Type - ABB IRB 120:
    Absolute Joint Position:
        Joint 1: [+/- 165.0] [°]
        Joint 2: [+/- 110.0] [°]
        Joint 3: [-110.0, +70.0] [°]
        Joint 4: [+/- 160.0] [°]
        Joint 5: [+/- 120.0] [°]
        Joint 6: [+/- 400.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [  0.0, -1.57,   0.0,   0.0,   0.0,   0.0]
        a          = [  0.0, 0.270,  0.07,   0.0,   0.0,   0.0]
        d          = [0.290,   0.0,   0.0, 0.302,   0.0, 0.072]
        alpha      = [-1.57,   0.0, -1.57,  1.57, -1.57,   0.0]
"""
ABB_IRB_120_Str = Robot_Parameters_Str(Name = 'ABB_IRB_120_ID_001')
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_120_Str.T.Base = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0, 0.0],
                                  [0.0, 0.0, 0.0, 1.0]], np.float32)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_120_Str.T.End_Effector = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                          [0.0, 1.0, 0.0, 0.0],
                                          [0.0, 0.0, 1.0, 0.0],
                                          [0.0, 0.0, 0.0, 1.0]], np.float32)
# Denavit-Hartenberg (DH)
ABB_IRB_120_Str.DH.Standard = np.array([[0.0,                   0.0, 0.290, -1.5707963267948966],
                                        [-1.5707963267948966, 0.270,   0.0,                 0.0],
                                        [0.0,                  0.07,   0.0, -1.5707963267948966],
                                        [0.0,                   0.0, 0.302,  1.5707963267948966],
                                        [0.0,                   0.0,   0.0, -1.5707963267948966],
                                        [3.141592653589793,     0.0, 0.072,                 0.0]], dtype = np.float32)

ABB_IRB_120_Str.DH.Modified = np.array([[0.0,                   0.0, 0.290,                 0.0],
                                        [-1.5707963267948966,   0.0,   0.0, -1.5707963267948966],
                                        [0.0,                 0.270,   0.0,                 0.0],
                                        [0.0,                  0.07, 0.302, -1.5707963267948966],
                                        [0.0,                   0.0,   0.0,  1.5707963267948966],
                                        [3.141592653589793,     0.0, 0.072, -1.5707963267948966]], dtype = np.float32)
# Zero/Home absolute position of each joint.
ABB_IRB_120_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float32)
ABB_IRB_120_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 90.0, 0.0], 
                                                                   dtype=np.float64))
# Limits of absolute joint position.
ABB_IRB_120_Str.Theta.Limit = np.array([[-2.8797932657906435, 2.8797932657906435], 
                                        [-1.9198621771937625, 1.9198621771937625], 
                                        [-1.9198621771937625, 1.2217304763960306], 
                                        [ -2.792526803190927,  2.792526803190927], 
                                        [-2.0943951023931953, 2.0943951023931953], 
                                        [ -3.141592653589793,  3.141592653589793]], dtype = np.float32)
# Other parameters of the robot structure.
ABB_IRB_120_Str.Theta.Name = [f'Joint_1_{ABB_IRB_120_Str.Name}', f'Joint_2_{ABB_IRB_120_Str.Name}', 
                              f'Joint_3_{ABB_IRB_120_Str.Name}', f'Joint_4_{ABB_IRB_120_Str.Name}', 
                              f'Joint_5_{ABB_IRB_120_Str.Name}', f'Joint_6_{ABB_IRB_120_Str.Name}']
ABB_IRB_120_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_120_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z']

"""
ABB_IRB_120_Str.DH.Modified = np.array([[0.0,                   0.0, 0.113,                 0.0],
                                              [0.0,                   0.0, 0.290,                 0.0],
                                              [-1.5707963267948966,   0.0,   0.0, -1.5707963267948966],
                                              [0.0,                 0.270,   0.0,                 0.0],
                                              [0.0,                  0.07, 0.302, -1.5707963267948966],
                                              [0.0,                   0.0,   0.0,  1.5707963267948966],
                                              [3.141592653589793,     0.0, 0.072, -1.5707963267948966]], dtype = np.float32)

# Zero absolute position of each joint.
ABB_IRB_120_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float32)
# Limits of absolute joint position.
ABB_IRB_120_Str.Theta.Limit = np.array([[-1.0, 1.0],
                                              [-2.8797932657906435, 2.8797932657906435], 
                                              [-1.9198621771937625, 1.9198621771937625], 
                                              [-1.9198621771937625, 1.2217304763960306], 
                                              [ -2.792526803190927,  2.792526803190927], 
                                              [-2.0943951023931953, 2.0943951023931953], 
                                              [ -3.141592653589793,  3.141592653589793]], dtype = np.float32)
# Parameters of the object (Blender robot arm).
ABB_IRB_120_Str.Theta.Name = ['Joint_0', 'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
ABB_IRB_120_Str.Theta.Type = ['P', 'R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_120_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z', 'Z']
"""

"""
Robot Type - ABB IRB 14000 (Right):
    Absolute Joint Position:
        Joint 1: [+/- 168.5] [°]
        Joint 2: [-143.5, +43.5] [°]
        Joint 7: [+/- 168.5] [°]
        Joint 3: [-123.5, +80.0] [°]
        Joint 4: [+/- 290.0] [°]
        Joint 5: [-88.0, +138.0] [°]
        Joint 6: [+/- 229.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [  0.0,    0.0,    0.0, -1.57,  3.14,    0.0,   0.0]
        a          = [0.030, -0.030,  0.040, 0.040, 0.027, -0.027,   0.0]
        d          = [  0.1,    0.0, 0.2515,   0.0, 0.265,    0.0, 0.036]
        alpha      = [-1.57,   1.57,  -1.57, -1.57, -1.57,  -1.57,   0.0]
"""
ABB_IRB_14000_R_Str = Robot_Parameters_Str(Name = 'ABB_IRB_14000_R_ID_001')
# Homogeneous transformation matrix of the base of the right arm (T_Base @ T_Base_R)
#   1\ Right Arm:
#       [[ 0.5713, -0.1071,  0.8138,  0.0536],
#        [ 0.6198,  0.7063, -0.3421, -0.0725],
#        [-0.5381,  0.6998,  0.4698,  0.4149],
#        [    0.0,     0.0,     0.0,     1.0]]
ABB_IRB_14000_R_Str.T.Base = HTM_Cls([[ 0.5713, -0.1071,  0.8138,  0.0536],
                                      [ 0.6198,  0.7063, -0.3421, -0.0725],
                                      [-0.5381,  0.6998,  0.4698,  0.4149],
                                      [    0.0,     0.0,     0.0,     1.0]], np.float32)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_14000_R_Str.T.End_Effector = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                              [0.0, 1.0, 0.0, 0.0],
                                              [0.0, 0.0, 1.0, 0.0],
                                              [0.0, 0.0, 0.0, 1.0]], np.float32)
# Denavit-Hartenberg (DH)
ABB_IRB_14000_R_Str.DH.Standard = np.array([[                0.0,  0.030,    0.1, -1.5707963267948966],
                                            [                0.0, -0.030,    0.0,  1.5707963267948966],
                                            [                0.0, 0.0405, 0.2515, -1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793,  0.027,  0.265, -1.5707963267948966],
                                            [                0.0, -0.027,    0.0,  1.5707963267948966],
                                            [                0.0,    0.0,  0.036,                 0.0]], dtype = np.float32)

ABB_IRB_14000_R_Str.DH.Modified = np.array([[                0.0,    0.0,    0.1,                 0.0],
                                            [                0.0,  0.030,    0.0, -1.5707963267948966],
                                            [                0.0, -0.030, 0.2515,  1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793, 0.0405,  0.265, -1.5707963267948966],
                                            [                0.0,  0.027,    0.0, -1.5707963267948966],
                                            [                0.0, -0.027,  0.036,  1.5707963267948966]], dtype = np.float32)
# Zero/Home absolute position of each joint.
ABB_IRB_14000_R_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                                          dtype = np.float32)
ABB_IRB_14000_R_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([0.0, -130.0, -135.0, 30.0, 0.0, 40.0, 0.0], 
                                                                       dtype=np.float64))
# Limits of absolute joint position.
ABB_IRB_14000_R_Str.Theta.Limit = np.array([[-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.504547476611863, 0.7592182246175333],
                                            [-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.155481626212997, 1.3962634015954636], 
                                            [ -5.061454830783556,  5.061454830783556], 
                                            [  -1.53588974175501, 2.4085543677521746], 
                                            [-3.9968039870670147, 3.9968039870670147]], dtype = np.float32)
# Other parameters of the robot structure.
ABB_IRB_14000_R_Str.Theta.Name = [f'R_Joint_1_{ABB_IRB_14000_R_Str.Name}', f'R_Joint_2_{ABB_IRB_14000_R_Str.Name}', 
                                  f'R_Joint_7_{ABB_IRB_14000_R_Str.Name}', f'R_Joint_3_{ABB_IRB_14000_R_Str.Name}', 
                                  f'R_Joint_4_{ABB_IRB_14000_R_Str.Name}', f'R_Joint_5_{ABB_IRB_14000_R_Str.Name}', 
                                  f'R_Joint_6_{ABB_IRB_14000_R_Str.Name}']
ABB_IRB_14000_R_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_14000_R_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z', 'Z']

"""
Robot Type - ABB IRB 14000 (Left):
    Absolute Joint Position:
        Joint 1: [+/- 168.5] [°]
        Joint 2: [-143.5, +43.5] [°]
        Joint 7: [+/- 168.5] [°]
        Joint 3: [-123.5, +80.0] [°]
        Joint 4: [+/- 290.0] [°]
        Joint 5: [-88.0, +138.0] [°]
        Joint 6: [+/- 229.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [  0.0,    0.0,    0.0, -1.57,  3.14,    0.0,   0.0]
        a          = [0.030, -0.030,  0.040, 0.040, 0.027, -0.027,   0.0]
        d          = [  0.1,    0.0, 0.2515,   0.0, 0.265,    0.0, 0.036]
        alpha      = [-1.57,   1.57,  -1.57, -1.57, -1.57,  -1.57,   0.0]
"""
ABB_IRB_14000_L_Str = Robot_Parameters_Str(Name = 'ABB_IRB_14000_L_ID_001')
# Homogeneous transformation matrix of the base of the left arm (T_Base @ T_Base_L)
#   1\ Left Arm:
#       [[ 0.5716,  0.1048, 0.8138, 0.0536],
#        [-0.6170,  0.7088, 0.3420, 0.0725],
#        [-0.5410, -0.6976, 0.4698, 0.4149],
#        [    0.0,     0.0,    0.0,    1.0]]
ABB_IRB_14000_L_Str.T.Base = HTM_Cls([[ 0.5716,  0.1048, 0.8138, 0.0536],
                                      [-0.6170,  0.7088, 0.3420, 0.0725],
                                      [-0.5410, -0.6976, 0.4698, 0.4149],
                                      [    0.0,     0.0,    0.0,    1.0]], np.float32)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_14000_L_Str.T.End_Effector = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                              [0.0, 1.0, 0.0, 0.0],
                                              [0.0, 0.0, 1.0, 0.0],
                                              [0.0, 0.0, 0.0, 1.0]], np.float32)
# Denavit-Hartenberg (DH)
ABB_IRB_14000_L_Str.DH.Standard = np.array([[                0.0,  0.030,    0.1, -1.5707963267948966],
                                            [                0.0, -0.030,    0.0,  1.5707963267948966],
                                            [                0.0, 0.0405, 0.2515, -1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793,  0.027,  0.265, -1.5707963267948966],
                                            [                0.0, -0.027,    0.0,  1.5707963267948966],
                                            [                0.0,    0.0,  0.036,                 0.0]], dtype = np.float32)
ABB_IRB_14000_L_Str.DH.Modified = np.array([[                0.0,    0.0,    0.1,                 0.0],
                                            [                0.0,  0.030,    0.0, -1.5707963267948966],
                                            [                0.0, -0.030, 0.2515,  1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793, 0.0405,  0.265, -1.5707963267948966],
                                            [                0.0,  0.027,    0.0, -1.5707963267948966],
                                            [                0.0, -0.027,  0.036,  1.5707963267948966]], dtype = np.float32)
# Zero/Home absolute position of each joint.
ABB_IRB_14000_L_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                                          dtype = np.float32)
ABB_IRB_14000_L_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([0.0, -130.0,  135.0, 30.0, 0.0, 40.0, 0.0], 
                                                                       dtype=np.float64))
# Limits of absolute joint position.
ABB_IRB_14000_L_Str.Theta.Limit = np.array([[-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.504547476611863, 0.7592182246175333],
                                            [-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.155481626212997, 1.3962634015954636], 
                                            [ -5.061454830783556,  5.061454830783556], 
                                            [  -1.53588974175501, 2.4085543677521746], 
                                            [-3.9968039870670147, 3.9968039870670147]], dtype = np.float32)
# Parameters of the object (Blender robot arm).
ABB_IRB_14000_L_Str.Theta.Name = [f'L_Joint_1_{ABB_IRB_14000_L_Str.Name}', f'L_Joint_2_{ABB_IRB_14000_L_Str.Name}', 
                                  f'L_Joint_7_{ABB_IRB_14000_L_Str.Name}', f'L_Joint_3_{ABB_IRB_14000_L_Str.Name}', 
                                  f'L_Joint_4_{ABB_IRB_14000_L_Str.Name}', f'L_Joint_5_{ABB_IRB_14000_L_Str.Name}', 
                                  f'L_Joint_6_{ABB_IRB_14000_L_Str.Name}']
ABB_IRB_14000_L_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_14000_L_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z', 'Z']

"""
Robot Type - Epson LS3-B401S:
    Absolute Joint Position:
        Joint 1: [-40, +220] [°]
        Joint 2: [+/- 140] [°]
        Joint 3: [-0.150, +0] [m]
        Joint 4: [+/- 180] [°]

    Denavit-Hartenberg (DH) Standard:
        Method 1 (th_3 - rotates counterclockwise):
            Note 1: The direction of the Z axis is upwards.
            Note 2: The Denavit-Hartenberg parameter d from 
                    the table will be positive (see Kinematics.py).
                theta_zero = [   0.0,    0.0, 0.0,     0.0]
                a          = [ 0.225,  0.175, 0.0,     0.0]
                d          = [0.1731, 0.0499, 0.0, -0.0785]
                alpha      = [   0.0,    0.0, 0.0,     0.0]
        Method 2 (th_3 - rotates clockwise):
            Note 1: The direction of the Z axis is downwards.
            Note 2: The Denavit-Hartenberg parameter d from 
                    the table will be negative (see Kinematics.py).
                theta_zero = [   0.0,    0.0, 0.0,    0.0]
                a          = [ 0.225,  0.175, 0.0,    0.0]
                d          = [0.1731, 0.0499, 0.0, 0.0785]
                alpha      = [   0.0,   3.14, 0.0,    0.0]
"""
EPSON_LS3_B401S_Str = Robot_Parameters_Str(Name = 'EPSON_LS3_B401S_ID_001')
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
EPSON_LS3_B401S_Str.T.Base = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                      [0.0, 1.0, 0.0, 0.0],
                                      [0.0, 0.0, 1.0, 0.0],
                                      [0.0, 0.0, 0.0, 1.0]], np.float32)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
EPSON_LS3_B401S_Str.T.End_Effector = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                              [0.0, 1.0, 0.0, 0.0],
                                              [0.0, 0.0, 1.0, 0.0],
                                              [0.0, 0.0, 0.0, 1.0]], np.float32)
# Denavit-Hartenberg (DH)
EPSON_LS3_B401S_Str.DH.Standard = np.array([[0.0, 0.225,  0.1731,               0.0],
                                            [0.0, 0.175,  0.0499, 3.141592653589793],
                                            [0.0,   0.0,     0.0,               0.0],
                                            [0.0,   0.0,  0.0785,               0.0]], dtype = np.float32) 
EPSON_LS3_B401S_Str.DH.Modified = np.array([[0.0,   0.0,  0.1731,               0.0],
                                            [0.0, 0.225,  0.0499,               0.0],
                                            [0.0, 0.175,     0.0, 3.141592653589793],
                                            [0.0,   0.0,  0.0785,               0.0]], dtype = np.float32) 
# Zero/Home absolute position of each joint.
EPSON_LS3_B401S_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0], 
                                          dtype = np.float32)
EPSON_LS3_B401S_Str.Theta.Home = np.array([Mathematics.Degree_To_Radian(90.0), Mathematics.Degree_To_Radian(0.0), 0.0, Mathematics.Degree_To_Radian(0.0)],
                                          dtype = np.float32)
# Limits of absolute joint position.
EPSON_LS3_B401S_Str.Theta.Limit = np.array([[-0.6981317007977318, 3.839724354387525], 
                                            [ -2.443460952792061, 2.443460952792061], 
                                            [             -0.150,               0.0], 
                                            [ -3.141592653589793, 3.141592653589793]], dtype = np.float32)
# Parameters of the object (Blender robot arm).
EPSON_LS3_B401S_Str.Theta.Name = [f'Joint_1_{EPSON_LS3_B401S_Str.Name}', f'Joint_2_{EPSON_LS3_B401S_Str.Name}', 
                                  f'Joint_34_{EPSON_LS3_B401S_Str.Name}', f'Joint_34_{EPSON_LS3_B401S_Str.Name}']
EPSON_LS3_B401S_Str.Theta.Type = ['R', 'R', 'P', 'R']
EPSON_LS3_B401S_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z']
