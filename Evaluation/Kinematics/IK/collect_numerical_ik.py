# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# Time (Time access and conversions)
import time
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core
#    ../RoLE/Trajectory/Utilities
import RoLE.Trajectory.Utilities
#   ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO as File_IO
#   Configuration
#       ./Configuration/Parameters
import Configuration.Parameters

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# Numerical IK Parameters.
#   Name of the numerical method to be used to calculate the IK solution.
#       'Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton', 
#       'Levenberg-Marquardt'
CONST_NIK_METHOD = 'Levenberg-Marquardt'
#   The properties of the inverse kinematics solver.
#       'tolerance': 1e-03 -> 'Jacobian-Transpose'
#       'tolerance': 1e-30 -> 'Newton-Raphson', 'Gauss-Newton', and 'Levenberg-Marquardt'
CONST_IK_PROPERTIES = {'delta_time': None, 'num_of_iteration': 500, 
                       'tolerance': 1e-30}

def main():
    """
    Description:
        A program to test the various methods of numerical inverse kinematics (IK) calculation 
        for individual robotic structures.

        Note:
            The test will be performed by generating a trajectory using a trapezoidal 
            profile, on which the calculation will be verified.
    """
    
    # Locate the path to the project folder.
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file will be saved.
    file_path = f'{project_folder}/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Remove the '*.txt' file if it already exists.
    for _, file_i in enumerate([f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Desired',
                                f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Predicted',
                                f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Absolute_Joint_Positions',
                                f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Error',
                                f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Iteration',
                                f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Time']):
      if os.path.isfile(f'{file_i}.txt'):
          os.remove(f'{file_i}.txt')

    # Initialization of the class to generate trajectory.
    Trapezoidal_Cls = RoLE.Trajectory.Utilities.Trapezoidal_Profile_Cls(delta_time=0.01)

    # Obtain the constraints for absolute joint positions in order to generate multi-axis position trajectories.
    (abs_j_pos_0, abs_j_pos_1) = Configuration.Parameters.Get_Absolute_Joint_Positions(Robot_Str.Name)

    # Obtain the desired homogeneous transformation matrix T of the tool center point (TCP).
    T_1 = RoLE.Kinematics.Core.Forward_Kinematics(abs_j_pos_1, 'Fast', Robot_Str)[1]

    # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
    #   IK:
    #       Theta <-- T
    (info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(T_1, abs_j_pos_0, CONST_NIK_METHOD, Robot_Str, 
                                                                      CONST_IK_PROPERTIES)

    # Check the calculation.
    if info["successful"] == False:
        print('[INFO] There is a problem with the IK calculation. Please modify the IK properties')
        sys.exit(0)
        
    # Generation of multi-axis position trajectories from input parameters.
    #   Note:
    #       Demonstration of joint interpolation instead of linear interpolation.
    theta_arr = []
    for _, (th_actual, th_desired) in enumerate(zip(abs_j_pos_0, theta)):
        (theta_arr_i, _, _) = Trapezoidal_Cls.Generate(th_actual, th_desired, 0.0, 0.0,
                                                       Configuration.Parameters.CONST_T_0, Configuration.Parameters.CONST_T_1)
        theta_arr.append(theta_arr_i)

    print('[INFO] The calculation is in progress.')

    # Calculation of inverse kinematics (IK) using the chosen numerical method.
    theta_0 = abs_j_pos_0.copy(); theta_T = np.array(theta_arr, dtype=np.float64).T
    for i, theta_arr_i in enumerate(theta_T):
        # Obtain the homogeneous transformation matrix of the robot end-effector from the input absolute joint positions.
        #   FK: 
        #       Theta --> T
        T_i = RoLE.Kinematics.Core.Forward_Kinematics(theta_arr_i, 'Fast', Robot_Str)[1]

        # Start time.
        t_0 = np.float64(time.time())

        # Obtain the absolute positions of the joints from the input homogeneous transformation matrix of the robot's end-effector.
        #   IK:
        #       Theta <-- T
        (info, theta_i) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(T_i, theta_0, CONST_NIK_METHOD, Robot_Str, 
                                                                            CONST_IK_PROPERTIES)
        
        # Stop time.
        t = np.float64(time.time() - t_0)

        # Check the calculation.
        if info["successful"] == False:
            print(f'[WARNING] The calculation of IK stopped in iteration {i}.')
            break

        # Obtain the translation and rotation part from the desired/predicted homogeneous 
        # transformation matrix.
        #   1\ Desired.
        p_i = T_i.p.all(); q_i = T_i.Get_Rotation('QUATERNION').all()
        #   2\ Predicted.
        T_predicted = RoLE.Kinematics.Core.Forward_Kinematics(theta_i, 'Fast', Robot_Str)[1]
        p_predicted = T_predicted.p.all(); q_predicted = T_predicted.Get_Rotation('QUATERNION').all()

        # Save the data to the '*.txt' file.
        File_IO.Save(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Desired', np.append(p_i, q_i), 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Predicted', np.append(p_predicted, q_predicted), 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Absolute_Joint_Positions', theta_i, 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Error', [info['error']['position'], 
                                                                                   info['error']['orientation'],
                                                                                   info['quadratic_error']], 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Iteration', [info["iteration"]], 'txt', ',')
        File_IO.Save(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Time', [t], 'txt', ',')
        
        # Obtain the last absolute position of the joint.
        theta_0 = theta_i.copy()

        # Release.
        t_0 = 0.0; t = 0.0

    # Display information.
    print(f'[INFO] The files have been successfully saved to the folder:')
    print(f'[INFO] >> {file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Desired.txt')
    print(f'[INFO] >> {file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Predicted.txt')
    print(f'[INFO] >> {file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Absolute_Joint_Positions.txt')
    print(f'[INFO] >> {file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Error.txt')
    print(f'[INFO] >> {file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Iteration.txt')
    print(f'[INFO] >> {file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Time.txt')

if __name__ == '__main__':
    sys.exit(main())