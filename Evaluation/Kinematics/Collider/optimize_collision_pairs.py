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
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#   ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_R_Str
# Number of randomly generated samples.
CONST_SIZE = 100000

def main():
    """
    Description:
        Program to optimize the collision pairs of the selected robotic arm.

        Note:
            The precision of the optimization will increase with the number 
            of randomly generated samples.
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

    # Obtain the initial number of collision pairs.
    n = (len(Robot_Str.Collider.Base) + \
         len(Robot_Str.Collider.Theta) + \
         len(Robot_Str.Collider.External)); r = 2

    # Get the number of possible combinations.
    C = Mathematics.Combinations(n - Robot_Str.Collider.Offset, r)
    print(f'[INFO] Initial number of collision pairs: {C}')

    # Save a list of the collision pairs.
    v = np.arange(0, n); collision_pairs_init = []
    for i, n_i in enumerate(v):
        for _, n_ij in enumerate(v[(i + 1) + Robot_Str.Collider.Offset::]):
            collision_pairs_init.append([n_i, n_ij])
    print(f'[INFO] >> Collision pairs ({C}, {r}): {collision_pairs_init}')
          
    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    """
    Description:
        Obtain information on whether there is a collision between the joints of the robotic structure.
    """
    collision_pairs_opt = []
    for _, th_rand_i in enumerate(theta_rand):
        # Get a list of base and joint colliders.
        Base_Collider = list(Robot_Str.Collider.Base.values()); Theta_Collider = list(Robot_Str.Collider.Theta.values())
        
        # Transformation of the base collider according to the input homogeneous transformation matrix.
        Base_Collider[0].Transformation(Robot_Str.T.Base)
    
        # Obtain the individual (theta) configuration of the homogeneous matrix of each joint using forward kinematics
        T_Arr = RoLE.Kinematics.Core.Get_Individual_Joint_Configuration(th_rand_i, 'Modified', Robot_Str)[1]

        # Transformation of the joint colliders according to the input homogeneous transformation matrix.
        for _, (T_i, th_collider_i) in enumerate(zip(T_Arr, Theta_Collider)):
            th_collider_i.Transformation(T_i)

        # Concatenate all colliders (base, joint) into single array according to a predefined constraint.
        if Robot_Str.External_Axis == True:
            Base_Collider[1].Transformation(T_Arr[0])
            All_Colliders = np.concatenate(([Base_Collider[0], Theta_Collider[0], Base_Collider[1]], 
                                            Theta_Collider[1::]))
        else:
            All_Colliders = np.concatenate((Base_Collider, Theta_Collider))

        # Check whether the 3D primitives (bounding boxes AABB, OBB) overlap or not.
        collision_pairs_opt_tmp = []
        for i, Collider_i in enumerate(All_Colliders):
            for j, Collider_j in enumerate(All_Colliders[(i + 1) + Robot_Str.Collider.Offset::], 
                                           start=(i + 1) + Robot_Str.Collider.Offset):
                if Collider_i.Overlap(Collider_j) == True:
                    # Store the individual parts where the collision occurs.
                    collision_pairs_opt_tmp.append([i, j])

        # Save the collision pairs if they are not already saved.
        if bool(collision_pairs_opt_tmp) == True:
            for _, c_pair_i in enumerate(collision_pairs_opt_tmp):
                if c_pair_i not in collision_pairs_opt:
                    collision_pairs_opt.append(c_pair_i)

    if bool(collision_pairs_opt) == True:
        # Sort the array of collision pairs by the first column.
        data = np.matrix(collision_pairs_opt)
        sorted_collision_pairs_opt = data[np.argsort(data.A[:, 0])]

        print(f'[INFO] Optimized number of collision pairs: {sorted_collision_pairs_opt.shape[0]}')
        print(f'[INFO] >> Collision pairs {sorted_collision_pairs_opt.shape}: {sorted_collision_pairs_opt.tolist()}')
    else:
        print(f'[WARNING] No collision pairs were found.')

    print('[INFO] The calculation process is complete.')
    print(f'[INFO] >> Time: {(time.time() - t_0):0.05f} in seconds.')

if __name__ == '__main__':
    sys.exit(main())
