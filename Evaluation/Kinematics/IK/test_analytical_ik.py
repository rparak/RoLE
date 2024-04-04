# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# Time (Time access and conversions)
import time
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core
#   Configuration
#       ../Configuration/Parameters
import Configuration.Parameters

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str

def main():
    """
    Description:
        A program to test the calculation of the inverse kinematics (IK) of the RRPR robotic 
        structure, also known as SCARA, using an analytical method.

        Two methods can be used to obtain IK solutions: 'All' or 'Best'.
            1\ 'All': Obtain the all possible solutions.
            2\ 'Best': Automatically obtain the best solution.
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
    (info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Analytical(T_1, abs_j_pos_0, Robot_Str, 'Best')

    t = time.time() - t_0
    print(f'[INFO] Time: {t:0.05f} in seconds.')

    # Display results.
    print(f'[INFO] Absolute Joint Positions:')
    print(f'[INFO] >> position_err = {info["error"]["position"]}, orientation_err = {info["error"]["orientation"]}')
    print(f'[INFO] >> theta = {theta}')
    print(f'[INFO] >> is_close_singularity = {info["is_close_singularity"]}')
    print(f'[INFO] >> is_self_collision = {info["is_self_collision"]}')

    # Check that the calculation has been performed successfully.
    accuracy = info['error']['position'] + info['error']['orientation']
    if accuracy <= 1e-5:
        print('[INFO] The IK solution test was successful.')
        print(f'[INFO] Accuracy = {accuracy}')
    else:
        print('[WARNING] A problem occurred during the calculation.')
        print(f'[INFO] Accuracy = {accuracy}')

if __name__ == '__main__':
    sys.exit(main())