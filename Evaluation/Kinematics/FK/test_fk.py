# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# Number of randomly generated samples.
CONST_SIZE = 1000

def main():
    """
    Description:
        A program to compare several approaches to calculate forward kinematics for an individual robotic arm. The program 
        also checks the correctness of the calculation.
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    """
    Description:
        Generation of random joint orientations in radians.
    """
    theta_rand_aux = [0.0] * len(Robot_Str.Theta.Limit)
    for i, th_limit in enumerate(Robot_Str.Theta.Limit):
        theta_rand_aux[i] = np.random.uniform(th_limit[0], th_limit[1], CONST_SIZE)
    
    theta_rand = np.transpose(theta_rand_aux)

    """
    Description:
        Time evaluation of different calculation methods.
    """
    print('[INFO] The calculation is in progress.')

    # 1\ Forward Kinematics (Standard)
    t_0 = time.time()
    for _, th in enumerate(theta_rand):
        RoLE.Kinematics.Core.Forward_Kinematics(th, 'Standard', Robot_Str)
    t = time.time() - t_0
    print(f'[INFO]  1\ Forward Kinematics (Standard)')
    print(f'[INFO]      Time: {t:0.05f} in seconds.')

    # 2\ Forward Kinematics (Modified)
    t_0 = time.time()
    for _, th in enumerate(theta_rand):
        RoLE.Kinematics.Core.Forward_Kinematics(th, 'Modified', Robot_Str)
    t = time.time() - t_0
    print(f'[INFO]  2\ Forward Kinematics (Modified)')
    print(f'[INFO]      Time: {t:0.05f} in seconds.')

    # 3\ Forward Kinematics (Fast)
    t_0 = time.time()
    for _, th in enumerate(theta_rand):
        RoLE.Kinematics.Core.Forward_Kinematics(th, 'Fast', Robot_Str)
    t = time.time() - t_0
    print(f'[INFO]  3\ Forward Kinematics (Fast)')
    print(f'[INFO]      Time: {t:0.05f} in seconds.')

    """
    Description:
        Evaluation of the correctness of calculations.
    """
    tolerance = 2
    for _, th in enumerate(theta_rand):
        T_Standard = np.round(np.array(RoLE.Kinematics.Core.Forward_Kinematics(th, 'Standard', Robot_Str)[1].all(), dtype=np.float64), 
                              tolerance)
        T_Modified = np.round(np.array(RoLE.Kinematics.Core.Forward_Kinematics(th, 'Modified', Robot_Str)[1].all(), dtype=np.float64), 
                              tolerance)
        T_Fast = np.round(np.array(RoLE.Kinematics.Core.Forward_Kinematics(th, 'Fast', Robot_Str)[1].all(), dtype=np.float64), 
                          tolerance)

        if (np.array_equal(T_Standard, T_Modified) and np.array_equal(T_Modified, T_Fast)) == False:
           # Warning: Something wrong!
           print('[WARNING] The resulting homogeneous transformation matrices are different!')
           print(f'[INFO] Forward Kinematics (Standard)')
           print(T_Standard)
           print(f'[INFO] Forward Kinematics (Modified)')
           print(T_Modified)
           print(f'[INFO] Forward Kinematics (Fast)')
           print(T_Fast)
           break

    print('[INFO] The calculation process is complete.')

if __name__ == '__main__':
    sys.exit(main())