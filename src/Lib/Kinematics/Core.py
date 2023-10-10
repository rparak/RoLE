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
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics
#   ../Lib/Transformation/Core
import Lib.Transformation.Core as Transformation

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
                    [          0.0,                                  0.0,                                    0.0,             1.0]], np.float64)

def __Forward_Kinematics_Standard(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
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
                    [                        0.0,                         0.0,                  0.0,                    1.0]], np.float64)

def __Forward_Kinematics_Modified(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using the modified Denavit-Hartenberg (DH) method.
        
        Note:
            DH (Denavit-Hartenberg) table: 
                theta (id: 0), a (id: 1), d (id: 2), alpha (id: 3)

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
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

    return T_i @ Robot_Parameters_Str.T.End_Effector

def Forward_Kinematics(theta: tp.List[float], method: str, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                              tp.List[tp.List[float]]]:
    """
    Description:
        Calculation of forward kinematics. The method of calculating depends on the input parameter (2).
        
    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
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
    th = (theta * Robot_Parameters_Str.Theta.Direction)

    return {
        'Standard': lambda th, th_err, r_param_str: (th_err, __Forward_Kinematics_Standard(th, r_param_str)),
        'Modified': lambda th, th_err, r_param_str: (th_err, __Forward_Kinematics_Modified(th, r_param_str)),
        'Fast': lambda th, th_err, r_param_str: (th_err, Utilities.FKFast_Solution(th, r_param_str))
    }[method](th, th_limit_err, Robot_Parameters_Str)

def __Get_Individual_Joint_Configuration_Standard(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[tp.List[float]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the standard forward kinematics calculation method.

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
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

    return T_cfg

def __Get_Individual_Joint_Configuration_Modified(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[tp.List[float]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the modified forward kinematics calculation method.

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
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

    return T_cfg

def Get_Individual_Joint_Configuration(theta: tp.List[float], method: str, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                                              tp.List[tp.List[tp.List[float]]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using forward kinematics. The method of calculating 
        the forward kinematics depends on the input parameter (2).

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
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
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 6xn]: Matrix of the geometric Jacobian (6 x n).
                                            Note: 
                                                Where n is equal to the number of joints.
    """
    
    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    # Get the configuration of the homogeneous transformation matrix of each joint using the 
    # modified forward kinematics calculation method.
    T_Cfg_Arr = __Get_Individual_Joint_Configuration_Modified(th, Robot_Parameters_Str)

    # Get the translation part from the homogeneous transformation matrix 
    # of the end-effector.
    T_n_p_ee = T_Cfg_Arr[-1].p

    J = np.zeros((6, th.size), dtype=T_n_p_ee.Type); z_i = Vector3_Cls(None, T_n_p_ee.Type)
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

def __Modify_IKN_Parameters(name: str, J: tp.List[tp.List[float]], e_i: tp.List[float]) -> tp.Tuple[tp.List[tp.List[float]],
                                                                                                    tp.List[float]]:
    """
    Description:
        Modification of the inverse kinematics (numerical) parameters, such as Jacobian and angle-axis error shape with 
        respect to the number of joints of the robotic manipulator.

    Args:
        (1) name [string]: Name of the robotic structure.
        (2) J [Matrix<float> 6xn]: Matrix of the geometric Jacobian (6 x n).
                                    Note: 
                                        Where n is equal to the number of joints 
        (3) e_i [Vector<float> 1x6]: Vector of an error (angle-axis).

    Returns:
        (1) parameter [Matrix<float> kxn]: Matrix of the modified geometric Jacobian (6 x n).
                                            Note: 
                                                Where k is equal to the number of axes and n is equal 
                                                to the number of joints.
        (2) parameter [Vector<float> 1xk]: Modified vector of an error (angle-axis).
                                            Note: 
                                                Where k is equal to the number of axes.
    """

    if name in ['Universal_Robots_UR3', 'ABB_IRB_120']:
        return (J.copy(), e_i.copy())
    else:
        return {
            'ABB_IRB_120_L_Ax': None,
            'ABB_IRB_14000_R': None,
            'ABB_IRB_14000_L': None,
            'EPSON_LS3_B401S': lambda J_in, e_i_in: (np.delete(J_in.copy(), [3, 4], axis=0), 
                                                     np.delete(e_i_in.copy(), [3, 4], axis=0))
        }[name](J, e_i)

def Inverse_Kinematics_Numerical_NR(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str, 
                                    ik_solver_properties: tp.Dict) -> tp.Tuple[tp.Dict, tp.List[float]]:
    """
    Description:
        A function to compute the inverse kinematics (IK) solution of the individual robotic structure using a numerical method 
        called Newton-Raphson (NR).

        Maybe add a dt time to interpolate ...
        
        Equation:
            ...

        # https://github.com/jhavl/dkt/blob/main/Part%201/4%20Numerical%20Inverse%20Kinematics.ipynb
    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (4) ik_solver_properties [Dictionary {'num_of_iteration': float, 
                                              'tolerance': float}]: The properties of the inverse kinematics solver.
                                                                        Note:
                                                                            'num_of_iteration': The number of iterations.
                                                                            'tolerance': Minimum required tolerance.

    Returns:
        (1) parameter [Dictionary {'successful': bool, 'iteration': int, 'error': {'position': float, 'orientation': float}, 
                                   'quadratic_error': float, 'is_close_singularity': bool, 
                                   'is_self_collision': bool}]: Information on the best results that were found.
                                                                Note:
                                                                    'successful': Information on whether the result was found 
                                                                                  within the required tolerance.
                                                                    'iteration': Information about the iteration in which the best 
                                                                                 result was found.
                                                                    'error': Information about the absolute error (position, orientation)
                                                                    'quadratic_error': Information about the quadratic (angle-axis) error.
                                                                    'is_close_singularity': Information about whether the Jacobian matrix 
                                                                                            is close to singularity.
                                                                    'is_self_collision': Information about whether there are collisions 
                                                                                         between joints.
        (2) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where n is the number of joints. 
    """

    # Get the number of joints.
    n_joints = Robot_Parameters_Str.Theta.Zero.size

    # Diagonal weight matrix.
    W_e = np.diag(np.ones(n_joints))

    # Get the current TCP position of the robotic arm using Forward Kinematics (FK).
    (th_limit_err, TCP_Position_0) = Forward_Kinematics(theta_0, 'Fast', Robot_Parameters_Str)
    
    is_successful = False; th_i = theta_0.copy(); th_i_tmp = theta_0.copy()
    for iteration_i in range(ik_solver_properties['num_of_iteration']):
        # Get the matrix of the geometric Jacobian.
        J_tmp = Get_Geometric_Jacobian(th_i, Robot_Parameters_Str)

        # Get an error (angle-axis) vector which represents the translation and rotation.
        e_i_tmp = General.Get_Angle_Axis_Error(TCP_Position, TCP_Position_0) 

        # Modification of the Jacobian and angle-axis error shape with respect 
        # to the number of joints of the robotic manipulator.
        (J, e_i) = __Modify_IKN_Parameters(Robot_Parameters_Str.Name, J_tmp, e_i_tmp)

        # Get the quadratic (angle-axis) error which is weighted by the diagonal 
        # matrix W_e.
        E = General.Get_Quadratic_Angle_Axis_Error(e_i, W_e)

        if E < ik_solver_properties['tolerance']:
            is_successful = True
            break
        else:
            # Newton-Raphson (NR) method.
            th_i += np.linalg.pinv(J) @ e_i

        # Get the current TCP position of the robotic arm using Forward Kinematics (FK).
        (th_limit_err, TCP_Position_0) = Forward_Kinematics(th_i, 'Fast', Robot_Parameters_Str)

        # Check whether the desired absolute joint positions are within the limits.
        for i, th_limit_err_i in enumerate(th_limit_err):
            if th_limit_err_i == True:
                th_i[i] = th_i_tmp[i]
            else:
                th_i_tmp[i] = th_i[i]

    # Check whether the absolute positions of the joints are close to a singularity or if there are collisions 
    # between the joints.
    is_close_singularity = General.Is_Close_Singularity(J)
    is_self_collision = General.Is_Self_Collision(th_i, Robot_Parameters_Str).any()

    # Obtain the absolute error of position and orientation.
    error = {'position': np.round(Mathematics.Euclidean_Norm((TCP_Position.p - TCP_Position_0.p).all()), 5), 
             'orientation': np.round(TCP_Position.Get_Rotation('QUATERNION').Distance('Euclidean', TCP_Position_0.Get_Rotation('QUATERNION')), 5)}
    
    # Write all the information about the results of the IK solution.
    return ({'successful': is_successful, 'iteration': iteration_i, 'error': error, 'quadratic_error': E, 
             'is_close_singularity': is_close_singularity, 'is_self_collision': is_self_collision}, th_i)

def Inverse_Kinematics_Numerical_GN(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str, 
                                    ik_solver_properties: tp.Dict) -> tp.Tuple[tp.Dict, tp.List[float]]:
    """
    Description:
        A function to compute the inverse kinematics (IK) solution of the individual robotic structure using a numerical method 
        called Gauss-Newton (GN).

        Equation:
            ...

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (4) ik_solver_properties [Dictionary {'num_of_iteration': float, 
                                              'tolerance': float}]: The properties of the inverse kinematics solver.
                                                                        Note:
                                                                            'num_of_iteration': The number of iterations.
                                                                            'tolerance': Minimum required tolerance.

    Returns:
        (1) parameter [Dictionary {'successful': bool, 'iteration': int, 'error': {'position': float, 'orientation': float}, 
                                   'quadratic_error': float}]: Information on the best results that were found.
                                                                Note:
                                                                    'successful': Information on whether the result was found 
                                                                                  within the required tolerance.
                                                                    'iteration': Information about the iteration in which the best 
                                                                                 result was found.
                                                                    'error': Information about the absolute error (position, orientation)
                                                                    'quadratic_error': Information about the quadratic (angle-axis) error.
                                   
                                   Information on whether the result was found within the required tolerance 
                                                                                         and information about the absolute error (position, orientation).
        (2) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where n is the number of joints. 
    """

    pass

def Inverse_Kinematics_Numerical_LM(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str, 
                                    ik_solver_properties: tp.Dict) -> tp.Tuple[tp.Dict, tp.List[float]]:
    """
    Description:
        A function to compute the inverse kinematics (IK) solution of the individual robotic structure using a numerical method 
        called Levenberg-Marquardt (LM).

        Equation:
            ...

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (4) ik_solver_properties [Dictionary {'num_of_iteration': float, 
                                              'tolerance': float}]: The properties of the inverse kinematics solver.
                                                                        Note:
                                                                            'num_of_iteration': The number of iterations.
                                                                            'tolerance': Minimum required tolerance.

    Returns:
        (1) parameter [Dictionary {'successful': bool, 'iteration': int, 'error': {'position': float, 'orientation': float}, 
                                   'quadratic_error': float}]: Information on the best results that were found.
                                                                Note:
                                                                    'successful': Information on whether the result was found 
                                                                                  within the required tolerance.
                                                                    'iteration': Information about the iteration in which the best 
                                                                                 result was found.
                                                                    'error': Information about the absolute error (position, orientation)
                                                                    'quadratic_error': Information about the quadratic (angle-axis) error.
                                   
                                   Information on whether the result was found within the required tolerance 
                                                                                         and information about the absolute error (position, orientation).
        (2) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where n is the number of joints. 
    """

    pass

def Inverse_Kinematics_Numerical(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], method: str, 
                                 Robot_Parameters_Str: Parameters.Robot_Parameters_Str, ik_solver_properties: tp.Dict) -> tp.Tuple[tp.Dict, tp.List[float]]:
    """
    Description:
        A function to compute the inverse kinematics (IK) solution of the individual robotic structure using a numerical method.

        Possible numerical methods that can be used include:
            1\ Newton-Raphson (NR) Method
            2\ Gauss-Newton (GN) Method
            3\ Levenberg-Marquardt (LM) Method

        Reference DKT:
            https://github.com/jhavl/dkt

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) method [string]: Name of the numerical method to be used to calculate the IK solution.
                                Note:
                                    method = 'Newton-Raphson'
        (4) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (5) ik_solver_properties [Dictionary {'num_of_iteration': float, 
                                              'tolerance': float}]: The properties of the inverse kinematics solver.
                                                                        Note:
                                                                            'num_of_iteration': The number of iterations.
                                                                            'tolerance': Minimum required tolerance.

    Returns:
        (1) parameter [Dictionary {'successful': bool, 'iteration': int, 'error': {'position': float, 'orientation': float}, 
                                   'quadratic_error': float}]: Information on the best results that were found.
                                                                Note:
                                                                    'successful': Information on whether the result was found 
                                                                                  within the required tolerance.
                                                                    'iteration': Information about the iteration in which the best 
                                                                                 result was found.
                                                                    'error': Information about the absolute error (position, orientation)
                                                                    'quadratic_error': Information about the quadratic (angle-axis) error.
        (2) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where n is the number of joints. 
    """

    try:
        assert method in ['Newton-Raphson', 'Gauss-Newton', 'Levenberg-Marquardt']

        if isinstance(TCP_Position, (list, np.ndarray)):
            TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(TCP_Position, np.float64)

        return {
            'Newton-Raphson': lambda tcp_p, th_0, r_param_str, ik_properties: Inverse_Kinematics_Numerical_NR(tcp_p, th_0, r_param_str, ik_properties),
            'Gauss-Newton': lambda tcp_p, th_0, r_param_str, ik_properties: Inverse_Kinematics_Numerical_GN(tcp_p, th_0, r_param_str, ik_properties),
            'Levenberg-Marquardt': lambda tcp_p, th_0, r_param_str, ik_properties: Inverse_Kinematics_Numerical_LM(tcp_p, th_0, r_param_str, ik_properties)
        }[method](TCP_Position, theta_0, Robot_Parameters_Str, ik_solver_properties)

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] An incorrect name of the method was selected for the numerical inverse kinematics calculation.')

def Inverse_Kinematics_Analytical(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], 
                                  Robot_Parameters_Str: Parameters.Robot_Parameters_Str, method: str) -> tp.Tuple[tp.Dict[tp.Union[float, tp.List[float]], 
                                                                                                                          tp.Union[float, tp.List[float]]], 
                                                                                                                  tp.Union[tp.List[float], tp.List[tp.List[float]]]]:
    """
    Description:
        A function to compute the solution of the inverse kinematics (IK) of the RRPR robotic structure (called SCARA) using an analytical method.

        Note:
            R - Revolute, P - Prismatic.

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (4) method [string]: The method chosen to obtain the solution of the absolute position of the joint.
                                Note:
                                    method = 'All' -> Obtain the all possible solutions.
                                    method = 'Best' -> Automatically obtain the best solution.
 
    Returns:
        (1) parameter [Dictionary {'error': {'position': float, 'orientation': float}, 'is_close_singularity': bool, 
                                   'is_self_collision': bool}]: Information on the best results that were found.
                                                                Note:
                                                                    'error': Information about the absolute error (position, orientation)
                                                                    'is_close_singularity': Information about whether the Jacobian matrix 
                                                                                            is close to singularity.
                                                                    'is_self_collision': Information about whether there are collisions 
                                                                                         between joints.
        (2) parameter [Matrix<float> kxn]: Obtained solution of the absolute position of the joint in radians / meters.
                                            Note:
                                                Where k is the number of solutions and n is the number of connections.
    """
        
    try:
        assert Robot_Parameters_Str.Name in ['EPSON_LS3_B401S']

        if isinstance(TCP_Position, (list, np.ndarray)):
            TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(TCP_Position, np.float64)

        # Initialization of output solutions.
        theta_solutions = np.zeros((2, Robot_Parameters_Str.Theta.Zero.size), dtype=np.float64)

        # Get the translational and rotational part from the transformation matrix.
        p = TCP_Position.p; Euler_Angles = TCP_Position.Get_Rotation('ZYX')

        """
        Calculation angle of Theta 1, 2 (Inverse trigonometric functions):
            Rule 1: 
                The range of the argument 'x' for arccos function is limited from -1 to 1.
                    -1 <= x <= 1
            Rule 2: 
                Output of arccos is limited from 0 to PI (radian).
                    0 <= y <= PI

        Auxiliary Calculations.
            Pythagorean theorem:
                L = sqrt(x^2 + y^2)
            Others:
                tan(gamma) = y/x -> gamma = arctan2(y, x)
        """

        # The Law of Cosines.
        #   L_{2}^2 = L_{1}^2 + L^2 - 2*L_{1}*L*cos(beta)
        #       ...
        #   cos(beta) = (L_{1}^2 + L^2 - L_{2}^2) / (2*L_{1}*L)
        #       ...
        #   beta = arccos((L_{1}^2 + L^2 - L_{2}^2) / (2*L_{1}*L))
        beta = ((Robot_Parameters_Str.DH.Modified[1, 1]**2) + (p.x**2 + p.y**2) - (Robot_Parameters_Str.DH.Modified[2, 1]**2)) \
              / (2*Robot_Parameters_Str.DH.Modified[1, 1]*np.sqrt(p.x**2 + p.y**2))
 
        # Calculation of the absolute position of the Theta_{1} joint.
        if beta > 1:
            theta_solutions[0, 0] = Robot_Parameters_Str.Theta.Direction[0] * np.arctan2(p.y, p.x) 
        elif beta < -1:
            theta_solutions[0, 0] = Robot_Parameters_Str.Theta.Direction[0] * (np.arctan2(p.y, p.x) - Mathematics.CONST_MATH_PI)
        else:
            # Configuration 1:
            #   cfg_{1} = gamma - beta 
            theta_solutions[0, 0] = Robot_Parameters_Str.Theta.Direction[0] * (np.arctan2(p.y, p.x) - np.arccos(beta))
            # Configuration 2:
            #   cfg_{2} = gamma + beta 
            theta_solutions[1, 0] = Robot_Parameters_Str.Theta.Direction[0] * (np.arctan2(p.y, p.x) + np.arccos(beta))
                
        # The Law of Cosines.
        #   L^2 = L_{1}^2 + L_{2}^2 - 2*L_{1}*L{2}*cos(alpha)
        #       ...
        #   cos(alpha) = (L_{1}^2 + L_{2}^2 - L^2) / 2*L_{1}*L{2}
        #       ...
        #   alpha = arccos((L_{1}^2 + L_{2}^2 - L^2) / 2*L_{1}*L{2})
        alpha = ((Robot_Parameters_Str.DH.Modified[1, 1]**2) + (Robot_Parameters_Str.DH.Modified[2, 1]**2) - (p.x**2 + p.y**2)) \
               / (2*(Robot_Parameters_Str.DH.Modified[1, 1]*Robot_Parameters_Str.DH.Modified[2, 1]))

        # Calculation of the absolute position of the Theta_{2} joint.
        if alpha > 1:
            theta_solutions[0, 1] = Robot_Parameters_Str.Theta.Direction[1] * Mathematics.CONST_MATH_PI
        elif alpha < -1:
            theta_solutions[0, 1] = 0.0
        else:
            # Configuration 1:
            #   cfg_{1} = PI - alpha
            theta_solutions[0, 1] = Robot_Parameters_Str.Theta.Direction[1] * (Mathematics.CONST_MATH_PI - np.arccos(alpha))
            # Configuration 2:
            #   cfg_{2} = alpha - PI
            theta_solutions[1, 1] = Robot_Parameters_Str.Theta.Direction[1] * (np.arccos(alpha) - Mathematics.CONST_MATH_PI)


        # Calculation of the absolute position of the Theta_{3} joint.
        theta_solutions[0, 2] =  Robot_Parameters_Str.Theta.Direction[2] * (p.z - (Robot_Parameters_Str.DH.Modified[0, 2] + 
                                                                                   Robot_Parameters_Str.DH.Modified[1, 2] - 
                                                                                   Robot_Parameters_Str.DH.Modified[3, 2]))
        theta_solutions[1, 2] = theta_solutions[0, 2].copy()

        # Calculation of the absolute position of the Theta_{4} joint.
        #   Method 1:  
        #       tan(theta_{1+2-4}) = sin(theta_{1-2-4}) / cos(theta_{1-2-4})
        #       
        #       We can obtain the sine and cosine functions from the homogeneous transformation 
        #       matrix.
        #           sin(theta_{1-2-4}) = TCP_Position.R[1, 0]
        #           cos(theta_{1-2-4}) = -TCP_Position.R[1, 1]
        #       
        #       theta_solutions[cfg_{i}, 3] = ... - np.arctan2(TCP_Position.R[1, 0], -TCP_Position.R[1, 1])
        #   Method 2:
        #       We can use the obtained Euler angles from the homogeneous transformation matrix.
        theta_solutions[0, 3] = Robot_Parameters_Str.Theta.Direction[3] * (theta_solutions[0, 0] + theta_solutions[0, 1] + Euler_Angles.z)
        theta_solutions[1, 3] = Robot_Parameters_Str.Theta.Direction[3] * (theta_solutions[1, 0] + theta_solutions[1, 1] + Euler_Angles.z)

        if method == 'All':
            info = {'error': {'position': np.zeros(theta_solutions.shape[0], dtype=np.float64), 
                              'orientation': np.zeros(theta_solutions.shape[0], dtype=np.float64)}, 
                              'is_close_singularity': np.zeros(theta_solutions.shape[0], dtype=bool), 
                              'is_self_collision': np.zeros(theta_solutions.shape[0], dtype=bool)}
            
            for i, th_sol_i in enumerate(theta_solutions):
                # Get the homogeneous transformation matrix of the robot end-effector from the input 
                # absolute joint positions.
                T = Forward_Kinematics(th_sol_i, 'Fast', Robot_Parameters_Str)[1]

                # Get the matrix of the geometric Jacobian.
                J_tmp = Get_Geometric_Jacobian(th_sol_i, Robot_Parameters_Str)

                # Modification of the Jacobian with respect to the number of joints of the 
                # robotic manipulator.
                J = np.delete(J_tmp.copy(), [3, 4], axis=0)

                # Check whether the absolute positions of the joints are close to a singularity or if there are collisions 
                # between the joints.
                info['is_close_singularity'][i] = General.Is_Close_Singularity(J)
                info['is_self_collision'][i] = General.Is_Self_Collision(th_sol_i, Robot_Parameters_Str).any()

                # Obtain the absolute error of position and orientation.
                info['error']['position'][i] = np.round(Mathematics.Euclidean_Norm((TCP_Position.p - T.p).all()), 5)
                info['error']['orientation'][i] = np.round(TCP_Position.Get_Rotation('QUATERNION').Distance('Euclidean', T.Get_Rotation('QUATERNION')), 5)

            return (info, theta_solutions)
        
        elif method == 'Best':
            # Automatically obtain the best solution for the absolute positions of the robot's joints.
            theta = General.Get_Best_IK_Solution(theta_0, theta_solutions, Robot_Parameters_Str)

            # Get the homogeneous transformation matrix of the robot end-effector from the input 
            # absolute joint positions.
            T = Forward_Kinematics(theta, 'Fast', Robot_Parameters_Str)[1]
            
            # Get the matrix of the geometric Jacobian.
            J_tmp = Get_Geometric_Jacobian(theta, Robot_Parameters_Str)

            # Modification of the Jacobian with respect to the number of joints of the 
            # robotic manipulator.
            J = np.delete(J_tmp.copy(), [3, 4], axis=0)

            # Check whether the absolute positions of the joints are close to a singularity or if there are collisions 
            # between the joints.
            is_close_singularity = General.Is_Close_Singularity(J)
            is_self_collision = General.Is_Self_Collision(theta, Robot_Parameters_Str).any()

            # Obtain the absolute error of position and orientation.
            error = {'position': np.round(Mathematics.Euclidean_Norm((TCP_Position.p - T.p).all()), 5), 
                     'orientation': np.round(TCP_Position.Get_Rotation('QUATERNION').Distance('Euclidean', T.Get_Rotation('QUATERNION')), 5)}
    
            # Write all the information about the results of the IK solution.
            return ({'error': error, 'is_close_singularity': is_close_singularity, 'is_self_collision': is_self_collision}, 
                    theta)

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] An incorrect robot structure was selected for the analytical inverse kinematics calculation.')




