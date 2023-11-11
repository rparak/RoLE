# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
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
# Custom Lib.: Industrial Robotics Library for Everyone (IRLE)
#   ../IRLE/Parameters/Robot
import IRLE.Parameters.Robot as Parameters

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
    project_folder = os.getcwd().split('Open_Industrial_Robotics')[0] + 'Open_Industrial_Robotics'

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

    # Create a figure.
    figure = plt.figure()
    ax = figure.add_subplot(projection='3d')

    # Plot the robot's workspace dependent on the input data from the file.
    ax.plot(np.round(x, 4), np.round(y, 4), np.round(z, 4), 'o', linewidth=1, markersize=2.0, color = [0,0.9,0.3,1.0],
            label=f'3D Positions (x, y, z)')
    
    # Set parameters of the graph (plot).
    ax.set_title(f'The Workspace of a {Robot_Str.Theta.Zero.size}-axis robotic arm {Robot_Str.Name}', fontsize=25, pad=25.0)
    #   Limits.
    ax.set_xlim(np.minimum.reduce(x) - 0.1, np.maximum.reduce(x) + 0.1)
    ax.xaxis.pane.set_color((1.0, 1.0, 1.0, 1.0))
    ax.set_ylim(np.minimum.reduce(y) - 0.1, np.maximum.reduce(y) + 0.1)
    ax.yaxis.pane.set_color((1.0, 1.0, 1.0, 1.0))
    ax.set_zlim(np.minimum.reduce(z) - 0.1, np.maximum.reduce(z) + 0.1)
    ax.zaxis.pane.set_color((1.0, 1.0, 1.0, 1.0))
    #   Label.
    ax.set_xlabel(r'x-axis in meters', fontsize=15, labelpad=10); ax.set_ylabel(r'y-axis in meters', fontsize=15, labelpad=10) 
    ax.set_zlabel(r'z-axis in meters', fontsize=15, labelpad=10) 
    #   Set parameters of the visualization.
    ax.xaxis._axinfo['grid'].update({'linewidth': 0.15, 'linestyle': '--'})
    ax.yaxis._axinfo['grid'].update({'linewidth': 0.15, 'linestyle': '--'})
    ax.zaxis._axinfo['grid'].update({'linewidth': 0.15, 'linestyle': '--'})
    #   Set the Axes box aspect.
    ax.set_box_aspect(None, zoom=0.90)
    # Get handles and labels for the legend.
    handles, labels = plt.gca().get_legend_handles_labels()
    # Remove duplicate labels.
    legend = dict(zip(labels, handles))
    # Show the labels (legends) of the graph.
    ax.legend(legend.values(), legend.keys(), fontsize=10.0)

    # Show the result.
    plt.show()

if __name__ == "__main__":
    sys.exit(main())
