# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../..' + 'src' not in sys.path:
    sys.path.append('../../../../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# SciencePlots (Matplotlib styles for scientific plotting) [pip3 install SciencePlots]
import scienceplots
# Matplotlib (Visualization) [pip3 install matplotlib]
import matplotlib.pyplot as plt
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ..Lib/Utilities/File_IO
import Lib.Utilities.File_IO as File_IO

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str
# Save the data to a file.
CONST_SAVE_DATA = False

def main():
    """
    Description:
        ...
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Open_Industrial_Robotics')[0] + 'Open_Industrial_Robotics'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file will be saved.
    file_path = f'{project_folder}/src/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Read data from the file.
    data = File_IO.Load(f'{file_path}/Method_Analytical_IK_Error', 'txt', ',')

    # Get the number of TCP (Tool Center Point) targets.
    N = np.arange(0.0, len(data[:, 0]), 1)

    label = [r'$e_{p}$', r'$e_{q}$']
    for i, data_i in enumerate(data.T):
        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        ax.plot(N, data_i, 'x', color='#8d8d8d', linewidth=3.0, markersize=8.0, markeredgewidth=3.0, markerfacecolor='#8d8d8d', label=label[i])
        ax.plot(N, [np.mean(data_i)] * N.size, '--', color='#8d8d8d', linewidth=1.5, label=f'Mean Absolute Error')

        # Set parameters of the graph (plot).
        #   Set the x ticks.
        ax.set_xticks(np.arange(np.min(N) - 10, np.max(N) + 10, 10))
        #   Set the y ticks.
        tick_y = (np.max(data_i) - np.min(data_i))/10.0
        ax.set_yticks(np.arange(np.min(data_i) - tick_y, np.max(data_i) + tick_y, tick_y))
        #   Label
        ax.set_xlabel(r'Number of TCP (Tool Center Point) targets', fontsize=15, labelpad=10)
        ax.set_ylabel(f'Absolute error {label[i]} in millimeters', fontsize=15, labelpad=10) 
        #   Set parameters of the visualization.
        ax.grid(which='major', linewidth = 0.15, linestyle = '--')
        # Get handles and labels for the legend.
        handles, labels = plt.gca().get_legend_handles_labels()
        # Remove duplicate labels.
        legend = dict(zip(labels, handles))
        # Show the labels (legends) of the graph.
        ax.legend(legend.values(), legend.keys(), fontsize=10.0)

        # Display the results as the values shown in the console.
        print(f'[INFO] Iteration: {i}')
        print(f'[INFO] max(label{i}) = {np.maximum(data_i)} in mm')
        print(f'[INFO] min(label{i}) = {np.minimum(data_i)} in mm')
        print(f'[INFO] Mean Absolute Error = {np.mean(data_i)} in mm')

        if CONST_SAVE_DATA == True:
            # Set the full scree mode.
            plt.get_current_fig_manager().full_screen_toggle()

            # Save the results.
            plt.savefig(f'{project_folder}/images/IK/{Robot_Str.Name}/Method_Analyrtical_IK_Error_{label[i]}.png', 
                        format='png', dpi=300)
        else:
            # Show the result.
            plt.show()

if __name__ == "__main__":
    sys.exit(main())