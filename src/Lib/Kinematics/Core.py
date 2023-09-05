# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Kinematics/Utilities/Forward_Kinematics
import Lib.Kinematics.Utilities.Forward_Kinematics as Utilities
#   ../Lib/Kinematics/Utilities/General
import Lib.Kinematics.Utilities.General as General
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Vector3_Cls

"""
Description:
    FORWARD KINEMATICS (FK)
        Forward kinematics kinematics refers to the use of the robot's kinematic equations to calculate 
        the position of the end-effector from specified values of the joint orientations.

    INVERSE KINEMATICS (IK)
        Inverse kinematics deals with the problem of finding the required joint angles to produce a certain desired 
        position and orientation of the end-effector. 

    DH (Denavit-Hartenberg) PARAMETERS: 
        The Denavit - Hartenberg parameters are the four parameters associated with a particular 
        convention for attaching reference frames to the links of a spatial kinematic chain, or robot 
        manipulator.

        | theta | 
        Joint angle (Theta_i). Rotation part in radians.
            Description:
                The joint angle Theta_i is the angle from xhat_{i-1} to xhat_i, measured about the zhat_i - axis.
        | a | 
        Link length (a_i). Translation part in meters.
            Description:
                The length of the mutually perpendicular line, denoted by the scalar a_{i-1}, is called the link 
                length of link i-1. Despite its name, this link length does not necessarily correspond to the actual 
                length of the physical link. 

        | d | 
        Link offset (d_i). Translation part in meters.
            Description:
                The link offset d_i is the distance from the intersection of xhat_{i-1} and zhat_i  to the origin 
                of the link - i frame (the positive direction is defined to be along the zhat_i - axis). 

        | alpha |  
        Link twist (alpha_i). Rotation part in radians.
            Description:
                The link twist a_{i-1} is the angle from zhat_{i-1} to zhat_i, measured about xhat_{i-1}. 
"""

def DH_Standard(theta: float, a: float, d: float, alpha: float) -> tp.List[tp.List[float]]:
    """
    Description:
        Standard Denavit-Hartenberg (DH) method. 
        
    Args:
        (1 - 4) theta, a, d, alpha [float]: DH (Denavit-Hartenberg) parameters in the current episode.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix in the current episode.
    """
    
    return HTM_Cls([[np.cos(theta), (-1.0)*(np.sin(theta))*np.cos(alpha),            np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                    [np.sin(theta),          np.cos(theta)*np.cos(alpha), (-1.0)*(np.cos(theta))*(np.sin(alpha)), a*np.sin(theta)],
                    [          0.0,                        np.sin(alpha),                          np.cos(alpha),               d],
                    [          0.0,                                  0.0,                                    0.0,             1.0]], np.float32)

def __Forward_Kinematics_Standard(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                            tp.List[tp.List[float]]]:
    """
    Description:
        Calculation of forward kinematics using the standard Denavit-Hartenberg (DH) method.

        Note:
            DH (Denavit-Hartenberg) table: 
                theta (id: 0), a (id: 1), d (id: 2), alpha (id: 3)

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """

    T_i = Robot_Parameters_Str.T.Base
    for _, (th_i, dh_i, th_i_type, th_ax_i) in enumerate(zip(theta, Robot_Parameters_Str.DH.Standard, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using standard DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Standard(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_ax_i == 'Z':
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

    # T_Base @ T_n @ T_EE
    return T_i @ Robot_Parameters_Str.T.End_Effector

def DH_Modified(theta: float, a: float, d: float, alpha: float) -> tp.List[tp.List[float]]:
    """
    Description:
        Modified Denavit-Hartenberg Method.
        
    Args:
        (1 - 4) theta, a, d, alpha [float]: DH (Denavit-Hartenberg) parameters in the current episode.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix in the current episode.
    """
    
    return HTM_Cls([[np.cos(theta)              ,        (-1.0)*np.sin(theta),                  0.0,                      a],
                    [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), (-1.0)*np.sin(alpha), (-1.0)*np.sin(alpha)*d],
                    [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),        np.cos(alpha),        np.cos(alpha)*d],
                    [                        0.0,                         0.0,                  0.0,                    1.0]], np.float32)

def __Forward_Kinematics_Modified(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                            tp.List[tp.List[float]]]:
    """
    Description:
        Calculation of forward kinematics using the modified Denavit-Hartenberg (DH) method.
        
        Note:
            DH (Denavit-Hartenberg) table: 
                theta (id: 0), a (id: 1), d (id: 2), alpha (id: 3)

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
    
    T_i = Robot_Parameters_Str.T.Base
    for _, (th_i, dh_i, th_i_type, th_ax_i) in enumerate(zip(theta, Robot_Parameters_Str.DH.Modified, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using modified DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Modified(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_ax_i == 'Z':
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

    # T_Base @ T_n @ T_EE
    return T_i @ Robot_Parameters_Str.T.End_Effector

def Forward_Kinematics(theta: tp.List[float], method: str, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                              tp.List[tp.List[float]]]:
    """
    Description:
        Calculation of forward kinematics. The method of calculating depends on the input parameter (2).
        
    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) method [string]: Forward kinematics method (1: Standard, 2: Modified, 3: Fast).
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Vector<bool>]: The result is a vector of values with a warning if the limit 
                                      is exceeded. 
                                      Note:
                                        The value in the vector is "True" if the desired absolute 
                                        joint positions are within the limits, and "False" if they 
                                        are not.
        (2) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """

    # Check that the desired absolute joint positions are not out of limit.
    th_limit_err = General.Check_Theta_Limit(theta, Robot_Parameters_Str)

    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    return {
        'Standard': lambda th, th_err, r_param_str: (th_err, __Forward_Kinematics_Standard(th, r_param_str)),
        'Modified': lambda th, th_err, r_param_str: (th_err, __Forward_Kinematics_Modified(th, r_param_str)),
        'Fast': lambda th, th_err, r_param_str: (th_err, Utilities.FKFast_Solution(th, r_param_str))
    }[method](th, th_limit_err, Robot_Parameters_Str)

def __Get_Individual_Joint_Configuration_Standard(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                                            tp.List[tp.List[tp.List[float]]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the standard forward kinematics calculation method.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<float> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                               Note:
                                                Where n is the number of joints.
    """
    
    T_i = Robot_Parameters_Str.T.Base; T_cfg = []
    for i, (th_i, dh_i, th_i_type, th_ax_i) in enumerate(zip(theta, Robot_Parameters_Str.DH.Standard, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using standard DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Standard(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_ax_i == 'Z':
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

        # Addition of a homogeneous transformation matrix configuration in the current 
        # episode (joint absolute position i).
        if theta.size - 1 == i:
            T_cfg.append(T_i @ Robot_Parameters_Str.T.End_Effector)
        else:
            T_cfg.append(T_i)

    # T_i[]
    return T_cfg

def __Get_Individual_Joint_Configuration_Modified(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                                            tp.List[tp.List[tp.List[float]]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the modified forward kinematics calculation method.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<float> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                               Note:
                                                Where n is the number of joints.
    """
    
    T_i = Robot_Parameters_Str.T.Base; T_cfg = []
    for i, (th_i, dh_i, th_i_type, th_ax_i) in enumerate(zip(theta, Robot_Parameters_Str.DH.Modified, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using modified DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Modified(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_ax_i == 'Z':
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

        # Addition of a homogeneous transformation matrix configuration in the current 
        # episode (joint absolute position i).
        if theta.size - 1 == i:
            T_cfg.append(T_i @ Robot_Parameters_Str.T.End_Effector)
        else:
            T_cfg.append(T_i)

    # T_i[]
    return T_cfg

def Get_Individual_Joint_Configuration(theta: tp.List[float], method: str, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                                              tp.List[tp.List[tp.List[float]]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using forward kinematics. The method of calculating 
        the forward kinematics depends on the input parameter (2).

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) method [string]: Forward kinematics method (1: Standard, 2: Modified).
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Vector<bool>]: The result is a vector of values with a warning if the limit 
                                      is exceeded. 
                                      Note:
                                        The value in the vector is "True" if the desired absolute 
                                        joint positions are within the limits, and "False" if they 
                                        are not.
        (2) parameter [Matrix<float> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                                Note:
                                                    Where n is the number of joints.
    """
    
    # Check that the desired absolute joint positions are not out of limit.
    th_limit_err = General.Check_Theta_Limit(theta, Robot_Parameters_Str)

    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    return {
        'Standard': lambda th, th_err, r_param_str: (th_err, __Get_Individual_Joint_Configuration_Standard(th, r_param_str)),
        'Modified': lambda th, th_err, r_param_str: (th_err, __Get_Individual_Joint_Configuration_Modified(th, r_param_str))
    }[method](th, th_limit_err, Robot_Parameters_Str)

def Get_Geometric_Jacobian(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the matrix of the geometric Jacobian (6 x n), where n equals the number of joints.

        The geometric Jacobian (also called the fundamental Jacobian) directly establishes 
        the relationship between joint velocities and the end-effector linear (p_{ee}') and angular (omega_{ee})
        velocities.

        Linear Velocity of the End-Effector:
            p_{ee}' = J_{P}(theta) * theta'
        
        Angular Velocity of the End-Effector:
            omega_{ee} = J_{O}(theta) * theta'

        The Jacobian can be divided into 3x1 columns J_{P} and J_{O} vectors:
            J(theta) = [[J_{P}], = ....
                        [J_{O}]] 

            Revolute Joint = [[z_{i-1} x (p_{ee} - p_{i-1})],
                              [z_{i-1}]]

            Prismatic Joint = [[z_{i-1}],
                               [0.0]]

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 6xn]: Matrix of the geometric Jacobian (6 x n).
                                           Note: 
                                            n is equal to the number of joints
    """
    
    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    # Get the configuration of the homogeneous transformation matrix of each joint using the 
    # modified forward kinematics calculation method.
    T_Cfg_Arr = __Get_Individual_Joint_Configuration_Modified(th, Robot_Parameters_Str)[1]

    # Get the translation part from the homogeneous transformation matrix 
    # of the end-effector.
    T_n_p_ee = T_Cfg_Arr[-1].p

    J = np.zeros((6, th.size)); z_i = Vector3_Cls(None, T_n_p_ee.Type)
    for i, (T_Cfg_i, th_i_type) in enumerate(zip(T_Cfg_Arr, Robot_Parameters_Str.Theta.Type)):
        z_i[:] = T_Cfg_i[0:3, 2]
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            J_P = z_i.Cross(T_n_p_ee - T_Cfg_i.p)
            J_O = z_i
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            J_P = z_i
            J_O = Vector3_Cls([0.0, 0.0, 0.0], z_i.Type)

        # The Jacobian can be divided into 3x1 columns J_{P} and J_{O} vectors:
        J[0:3, i] = J_P.all()
        J[3:6, i] = J_O.all()

    return J

# ////////////////////
"""
ik_solver_properties -> *args or **kwargs
Link: https://www.programiz.com/python-programming/args-and-kwargs#:~:text=Python%20**kwargs&text=In%20the%20function%2C%20we%20use,parameter%20excluding%20double%20asterisk%20**%20.
"""

"""
DKT:
https://github.com/jhavl/dkt
"""
# ///////////////////
def __Inverse_Kinematics_Numerical_NR(TCP_Position, theta_0, Robot_Parameters_Str, ik_solver_properties):
    pass

def __Inverse_Kinematics_Numerical_GN(TCP_Position, theta_0, Robot_Parameters_Str, ik_solver_properties):
    pass

def __Inverse_Kinematics_Numerical_LM(TCP_Position, theta_0, Robot_Parameters_Str, ik_solver_properties):
    pass

def Inverse_Kinematics_Numerical(TCP_Position, theta_0, method, Robot_Parameters_Str, ik_solver_properties):
    """
    Description:
        ....

        ik_solver_properties = {'num_of_iteration': ..., 'tolerance': .., 'etc.'...}

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) method [string]: The name of the solver used to calculate the numerical method of inverse kinematics
        (4) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (5) ik_solver_properties [Dictionary]: The properties of the inverse kinematics solver.
                                                Note:
                                                    The properties depend on the specific method."

    Returns:
        (1) parameter [Dictionary {'successful': bool, 
                                   'angle_axis_error': float}]: Information on whether the result was found within the required tolerance 
                                                                and information on the minimum quadratic (angle-axis) error.
        (2) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where n is the number of joints.
    """

    return {
        'Newton-Raphson': lambda tcp_p, th_0, r_param_str, properties: __Inverse_Kinematics_Numerical_NR(tcp_p, th_0, r_param_str, properties),
        'Gauss-Newton': lambda tcp_p, th_0, r_param_str, properties: __Inverse_Kinematics_Numerical_GN(tcp_p, th_0, r_param_str, properties),
        'Levenberg-Marquardt': lambda tcp_p, th_0, r_param_str, properties: __Inverse_Kinematics_Numerical_LM(tcp_p, th_0, r_param_str, properties)
    }[method](TCP_Position, theta_0, Robot_Parameters_Str, ik_solver_properties)

def Inverse_Kinematics_Analytical(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[float]:
    """
    Description:
        ....

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.


    Returns:
        (1) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where n is the number of joints.
    """
        
    try:
        assert Robot_Parameters_Str.Name in ['EPSON_LS3_B401S']

        pass

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] An incorrect robot structure was selected for the analytical inverse kinematics calculation.')




