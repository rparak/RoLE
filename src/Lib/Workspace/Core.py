# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# OS (Operating system interfaces)
import os
# Custom Script:
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core

"""
Description: 
    Definition of constants.
"""
CONST_NONE_VALUE = -1

def Get_Number_of_Samples(name):
    """
    Description:
        Get the number of samples for joint orientation combinations to generate the workspace 
        of a robotic arm.

        Note:
            An odd number to generate the necessary numbers.

    Args:
        (1) name [string]: Name of the robotic structure.

    Returns:
        (1) parameter [Vector<int> 1 x n]: Array of samples for each joint to generate the robot workspace.
                                           Note:
                                            Where n is the number of joints.
    """
    return {
        'Universal_Robots_UR3': [30 + 1, 30 + 1, 30 + 1, 3, 3, 0],
        'ABB_IRB_120': [30 + 1, 30 + 1, 30 + 1, 3, 3, 0],
        'ABB_IRB_14000_R': [30 + 1, 30 + 1, 30 + 1, 10 + 1, 3, 0, 0],
        'ABB_IRB_14000_L': [30 + 1, 30 + 1, 30 + 1, 10 + 1, 3, 0, 0],
        'EPSON_LS3_B401S': [30 + 1, 30 + 1, 30 + 1, 0]
    }[name]

def Convert_Orientation_Data_To_String(data):
    """
    Description:
        Delete redundant input data of absolute joint orientation and convert to a string.

    Args:
        (1) data [Vector<float> 1 x n]: Absolute joint orientation data.
                                        Note:
                                            Where n is the number of joints.

    Returns:
        (1) parameter [Vector<string>]: Output string for writing data to a file.   
    """

    output_string = []

    for data_i in data:
        if data_i != CONST_NONE_VALUE:
            output_string.append(str(data_i)); output_string.append(',')
        else:
            output_string.pop(-1); output_string.append('\n')
            break

    if output_string[-1] == ',':
        output_string.pop(-1); output_string.append('\n')

    return output_string

def Generate_Absolute_Joint_Orientation(file_path, N, theta):
    """
    Description:
        Generate the absolute orientation of the joints to calculate the robot's workspace 
        and save this data to a file.

    Args:
        (1) file_path [string]: The specified path of the file with an extension '*.txt'.
        (2) N [int]: Number of combinations to generate.
        (3) theta [Vector<float>]: Joint orientations in radians.
    """

    # if the file exists, then remove it.
    if os.path.exists(file_path):
        os.remove(file_path)

    th_0, th_1, th_2, th_3, th_4, th_5, th_6 = theta

    # Initialization of data to show the process flow.
    counter = 0
    percentage_offset = N/10
    percentage_idx    = 1

    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    with open(file_path, 'a+') as f:
        # Cyclically writes joint data to a file.
        for th_0i in th_0:
            for th_1i in th_1:
                for th_2i in th_2:
                    for th_3i in th_3:
                        for th_4i in th_4:
                            for th_5i in th_5:
                                for th_6i in th_6:
                                    # Write the data (abs. joint orientation) to a file using 
                                    # the file object (f).
                                    f.writelines(Convert_Orientation_Data_To_String([th_0i, th_1i, th_2i,
                                                                                     th_3i, th_4i, th_5i,
                                                                                     th_6i]))
                                
                                    counter += 1
                                    if counter > (percentage_offset * percentage_idx):
                                        print(f'[INFO]  Percentage: {int(100 * float(counter)/float(N))} %')
                                        print(f'[INFO]  Time: {(time.time() - t_0):.3f} in seconds')
                                        percentage_idx += 1

    # Close the file.
    f.close()

    print(f'[INFO]  Percentage: {int(100 * float(counter)/float(N))} %')
    print(f'[INFO] The file is successfully saved')
    print(f'[INFO] Number of processed combinations: {counter}')
    if counter == N:
        print('[INFO] The calculation is completed successfully.')
    else:
        print('[WARNING] Insufficient number of combinations.')

    print(f'[INFO] Total Time: {(time.time() - t_0):.3f} in seconds')

def Generate_Workspace_XYZ(Robot_Str, file_path_in, file_path_out):
    """
    Description:
        Generate x, y, z positions of the workspace from the absolute orientation of the joints.

    Args:
        (1, 2) file_path_in, file_path_out [string]: The specified path of the file with an extension '*.txt'.
                                                     Note:
                                                        *_in : as input data file
                                                        *_out : as output data file
    """

    # if the file exists, then remove it.
    if os.path.exists(file_path_out):
        os.remove(file_path_out)

    # Initialization of data to show the process flow.
    counter = 0
    N = sum(1 for _ in open(file_path_in))
    percentage_offset = N/10
    percentage_idx    = 1

    print('[INFO] The calculation is in progress.')
    t_0 = time.time()
    with open(file_path_in, 'r') as f_in , \
         open(file_path_out, 'a+') as f_out:

        for line in f_in:
            # Read the current row and split the data.
            data = line.split(',')

            # Calculation of forward kinematics. Get the Homogeneous end-effector transformation matrix {T}.
            T = Lib.Kinematics.Core.Forward_Kinematics(np.float32(data), 'Modified', Robot_Str)[1]

            # Get the translation part from the homogeneous transformation matrix {T}.
            p = T.p.all()

            # Write the data (translation part p - x, y, z) to a file using 
            # the file object (f).
            f_out.writelines([str(p[0]), str(','), str(p[1]), str(','), str(p[2]), '\n'])

            counter += 1
            if counter > (percentage_offset * percentage_idx):
                print(f'[INFO]  Percentage: {int(100 * float(counter)/float(N))} %')
                print(f'[INFO]  Time: {(time.time() - t_0):.3f} in seconds')
                percentage_idx += 1

    # Close the files.
    f_in.close(); f_out.close()

    print(f'[INFO]  Percentage: {int(100 * float(counter)/float(N))} %')
    print(f'[INFO] The file is successfully saved')
    print(f'[INFO] Number of processed data: {counter}')
    if counter == N:
        print('[INFO] The calculation is completed successfully.')
    else:
        print('[WARNING] Insufficient number of combinations.')
    print(f'[INFO] Total Time: {(time.time() - t_0):.3f} in seconds')