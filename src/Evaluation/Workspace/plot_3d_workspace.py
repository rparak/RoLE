# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# OS (Operating system interfaces)
import os
# SciencePlots (Matplotlib styles for scientific plotting) [pip3 install SciencePlots]
import scienceplots
# Matplotlib (Visualization) [pip3 install matplotlib]
import matplotlib.pyplot as plt
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str
# The name of the input file and folder.
CONST_FILE_NAME = 'tool0_workspace_data'

def main():
    """
    Description:
        A program for visualization the workspace of an individual robot structure.

        Note:
            The structures of the robot are defined below:
                ../Parameters/Robot.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Industrial_Robots_Kinematics')[0] + 'Industrial_Robots_Kinematics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Set the parameters for the scientific style.
    plt.style.use(['science'])

    # Initialization of data to show the process flow.
    counter = 0
    N = sum(1 for _ in open(f'{project_folder}/src/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME}.txt'))
    percentage_offset = N/10
    percentage_idx    = 1

    # Initialization of the x, y, z workspace data array.
    x = []; y = []; z = []

    """
    Desription:
        Read x, y, z data from a file and write to an array.
    """
    start_time = time.time()
    with open(f'{project_folder}/src/Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME}.txt', 'r') as f:
        for line in f:
            # Read the current row and split the data.
            data = line.split(',')

            # Write the x, y, z data to an array.
            x.append(np.float64(data[0]))
            y.append(np.float64(data[1]))
            z.append(np.float64(data[2]))

            counter += 1
            if counter > (percentage_offset * percentage_idx):
                print(f'[INFO]  Percentage: {int(100 * float(counter)/float(N))} %')
                print(f'[INFO]  Time: {(time.time() - start_time):.3f} in seconds')
                percentage_idx += 1

    f.close()
    print(f'[INFO]  Percentage: {int(100 * float(counter)/float(N))} %')
    print(f'[INFO] Number of processed data: {counter}')
    if counter == N:
        print('[INFO] The calculation is completed successfully.')
    else:
        print('[WARNING] Insufficient number of combinations.')
    print(f'[INFO] Total Time: {(time.time() - start_time):.3f} in seconds')

    # Create figure for 3d projection
    figure = plt.figure()
    axis = figure.add_subplot(projection='3d')
    figure.suptitle(f'The Workspace of a {Robot_Str.Theta.Zero.size}-axis robot arm {Robot_Str.Name}', fontsize = 15)

    # Plot the workspace of a robot dependent on input data from a file.
    axis.plot(np.round(x, 10), np.round(y, 10), np.round(z, 10), 'o', linewidth=1, markersize=0.15, color = [0,0.9,0.3,0.25])

    # Axis Parameters:
    #   Limits:
    axis.set_xlim(np.minimum.reduce(x) - 0.1, np.maximum.reduce(x) + 0.1)
    axis.xaxis.pane.set_color((1.0, 1.0, 1.0, 1.0))
    axis.set_ylim(np.minimum.reduce(y) - 0.1, np.maximum.reduce(y) + 0.1)
    axis.yaxis.pane.set_color((1.0, 1.0, 1.0, 1.0))
    axis.set_zlim(np.minimum.reduce(z) - 0.1, np.maximum.reduce(z) + 0.1)
    axis.zaxis.pane.set_color((1.0, 1.0, 1.0, 1.0))
    #   Labels:
    axis.set_xlabel(r'x-axis in metres'); axis.set_ylabel(r'y-axis in metres')
    axis.set_zlabel(r'z-axis in metres')
    #   Grid:
    axis.xaxis._axinfo['grid'].update({'linewidth': 0.25, 'linestyle': '--'})
    axis.yaxis._axinfo['grid'].update({'linewidth': 0.25, 'linestyle': '--'})
    axis.zaxis._axinfo['grid'].update({'linewidth': 0.25, 'linestyle': '--'})
    #   Others:
    axis.set_aspect('auto')
    axis.set_box_aspect(aspect = (1,1,1))

    # Show the result.
    plt.show()

if __name__ == "__main__":
    sys.exit(main())
