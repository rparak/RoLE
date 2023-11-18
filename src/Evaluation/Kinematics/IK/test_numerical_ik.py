# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../..' + 'src' not in sys.path:
    sys.path.append('../../..')
# Time (Time access and conversions)
import time
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core
#   ../RoLE/Trajectory/Utilities
import RoLE.Trajectory.Utilities
#   ../Configuration/Parameters
import Configuration.Parameters

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_R_Str
# Numerical IK Parameters.
#   Name of the numerical method to be used to calculate the IK solution.
#       'Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton', 
#       'Levenberg-Marquardt'
CONST_NIK_METHOD = 'Newton-Raphson'
#   The properties of the inverse kinematics solver.
CONST_IK_PROPERTIES = {'delta_time': 0.1, 'num_of_iteration': 500, 
                       'tolerance': 1e-10}

def main():
    """
    Description:
        A program to test the various methods of numerical inverse kinematics (IK) calculation 
        for individual robotic structures.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # Obtain the desired homogeneous transformation matrix T of the tool center point (TCP).
    T_1 = RoLE.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]

    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(T_1, abs_j_pos_0, CONST_NIK_METHOD, Robot_Str, 
                                                                     CONST_IK_PROPERTIES)
    
    t = time.time() - t_0
    print(f'[INFO] Time: {t:0.05f} in seconds.')

    # Display results.
    print(f'[INFO] Absolute Joint Positions:')
    print(f'[INFO] >> successful = {info["successful"]}')
    print(f'[INFO] >> iteration = {info["iteration"]}')
    print(f'[INFO] >> position_err = {info["error"]["position"]}, orientation_err = {info["error"]["orientation"]}')
    print(f'[INFO] >> theta = {theta}')
    print(f'[INFO] >> is_close_singularity = {info["is_close_singularity"]}')
    print(f'[INFO] >> is_self_collision = {info["is_self_collision"]}')

    # Check that the calculation has been performed successfully.
    accuracy = info["error"]["position"] + info["error"]["orientation"]
    if info["successful"] == True:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')

if __name__ == '__main__':
    sys.exit(main())