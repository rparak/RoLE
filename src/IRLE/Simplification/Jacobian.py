# System (Default)
import sys
#   Add access if it is not in the system path.
sys.path.append('../..')
# Time (Time access and conversions)
import time
# Sympy (Symbolic mathematics) [pip3 install sympy]
import sympy as sp
# Custom Lib.: Industrial Robotics Library for Everyone (IRLE)
#   ../IRLE/Manipulator/Parameters
import IRLE.Parameters.Robot as Parameters

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str

def __DH_Modified(theta: float, a: float, d: float, alpha: float) -> sp.Matrix:
    """
    Description:
        Modified Denavit-Hartenberg Method.
        
    Args:
        (1 - 4) theta, a, d, alpha [float]: DH (Denavit-Hartenberg) parameters in the current episode.
        
    Returns:
        (1) parameter [Matrix<sympy> 4x4]: Homogeneous transformation matrix in the current episode.
    """
        
    return sp.Matrix([[sp.cos(theta)              ,        (-1.0)*sp.sin(theta),                  0.0,                      a],
                      [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), (-1.0)*sp.sin(alpha), (-1.0)*sp.sin(alpha)*d],
                      [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha),        sp.cos(alpha),        sp.cos(alpha)*d],
                      [                        0.0,                         0.0,                  0.0,                    1.0]])


def __Get_Individual_Joint_Configuration_Modified(theta: sp.symbols, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> sp.Matrix:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the modified forward kinematics calculation method.

    Args:
        (1) theta [Vector<sympy>]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<sympy> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                               Note:
                                                Where n is the number of joints.
    """
    
    T_i = sp.Matrix(sp.eye(4)); T_cfg = []
    for i, (th_i, dh_i, th_i_type, th_ax_i) in enumerate(zip(theta, Robot_Parameters_Str.DH.Modified, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using modified DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ __DH_Modified(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_ax_i == 'Z':
                T_i = T_i @ __DH_Modified(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ __DH_Modified(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

        # Addition of a homogeneous transformation matrix configuration in the current 
        # episode (joint absolute position i).
        T_cfg.append(sp.simplify(T_i))

    return T_cfg

def Get_Geometric_Jacobian(theta: sp.symbols, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> sp.Matrix:
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
        (1) theta [Vector<sympy> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<sympy> 6xn]: Matrix of the geometric Jacobian (6 x n).
                                           Note: 
                                            n is equal to the number of joints
    """
    
    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    # Get the configuration of the homogeneous transformation matrix of each joint using the 
    # modified forward kinematics calculation method.
    T_Cfg_Arr = __Get_Individual_Joint_Configuration_Modified(th, Robot_Parameters_Str)

    # Get the translation part from the homogeneous transformation matrix 
    # of the end-effector.
    T_n_p_ee = T_Cfg_Arr[-1][:3, 3]

    J = sp.Matrix(sp.zeros(6, th.size)); z_i = sp.Array([0.0, 0.0, 0.0])
    for i, (T_Cfg_i, th_i_type) in enumerate(zip(T_Cfg_Arr, Robot_Parameters_Str.Theta.Type)):
        z_i = T_Cfg_i[0:3, 2]
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            J_P = z_i.cross(T_n_p_ee - T_Cfg_i[:3, 3])
            J_O = z_i
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            J_P = z_i
            J_O = sp.Array([0.0, 0.0, 0.0])

        # The Jacobian can be divided into 3x1 columns J_{P} and J_{O} vectors:
        J[0:3, i] = sp.simplify(J_P)
        J[3:6, i] = sp.simplify(J_O)
             
    return sp.simplify(J)

def main():
    """
    Description:
        A program to simplify the solution of the Jacobian calculation. The results of the simplification 
        will be used to calculate the Jacobian faster.

        Note:
            The shape of the Jacobian matrix must be square. Rewrite the results depending 
            on the particular robotic structure.
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialize a string containing the symbol assigned with the variable.
    theta = [sp.symbols(f'theta[{i}]') for i in range(Robot_Str.Theta.Zero.size)]

    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    """
    Description:
        Obtain the simplified matrix of the geometric Jacobian (6 x n), where n is the number of joints.
    """
    J_simpl = Get_Geometric_Jacobian(theta, Robot_Str)

    print('[INFO] Code generation.')
    print('J = np.zeros((6, theta.size), dtype=np.float64)')

    for i, J_i_simpl in enumerate(J_simpl.tolist()):
        for j, J_ij_simpl in enumerate(J_i_simpl):
            # Replace (convert) the old value string to the new one.
            #   Note: Better conversion to standard form (copy + paste to function).
            J_ij_simpl_new = str(sp.nsimplify(J_ij_simpl, tolerance=1e-5, rational=True).evalf()).replace('sin', 'np.sin').replace('cos', 'np.cos')
            print(f'J[{i},{j}] = {J_ij_simpl_new}')

    print('[INFO] The simplification process is successfully completed!')
    print(f'[INFO] Total Time: {(time.time() - t_0):.3f} in seconds')

if __name__ == '__main__':
    sys.exit(main())