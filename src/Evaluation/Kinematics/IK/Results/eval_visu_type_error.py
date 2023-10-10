# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../..' + 'src' not in sys.path:
    sys.path.append('../../../..')
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
# ...
CONST_IK_METHOD = 'Analytical'
# Numerical IK Parameters.
#   Method.
#       'Newton-Raphson', 'Gauss-Newton', 'Levenberg-Marquardt'
CONST_NIK_METHOD = 'Newton-Raphson'
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

    if CONST_IK_METHOD == 'Analytical':
        data = File_IO.Load(f'{file_path}/Method_Analytical_IK_Error', 'txt', ',')
    elif CONST_IK_METHOD == 'Numerical':
        data = File_IO.Load(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_Error', 'txt', ',')

    # Create a figure.
    _, ax = plt.subplots()

    t = np.arange(0.0, len(data[:, 0]), 1)
    #ax.plot(t, data[:, 0], 'x', color='#d0d0d0', linewidth=3.0, markersize=8.0, markeredgewidth=3.0, markerfacecolor = '#ffffff', label='...')
    ax.plot(t, data[:, 1], 'x', color='#8d8d8d', linewidth=3.0, markersize=8.0, markeredgewidth=3.0, markerfacecolor = '#737373', label=r'$e_{q}$')
    ax.plot(t, [np.mean(data[:, 1])] * t.size, '--', color='#4c4c4c', linewidth=1.5, label='Average Precision')

    # Set parameters of the graph (plot).
    ax.set_title(f'...', fontsize=25, pad=25.0)
    #   Set the x ticks.
    ax.set_xticks(np.arange(np.min(t) - 10, np.max(t) + 10, 10))
    #   Label
    ax.set_xlabel(r'..', fontsize=15, labelpad=10); ax.set_ylabel(r'..', fontsize=15, labelpad=10) 
    #   Set parameters of the visualization.
    ax.grid(which='major', linewidth = 0.15, linestyle = '--')
    # Get handles and labels for the legend.
    handles, labels = plt.gca().get_legend_handles_labels()
    # Remove duplicate labels.
    legend = dict(zip(labels, handles))
    # Show the labels (legends) of the graph.
    ax.legend(legend.values(), legend.keys(), fontsize=10.0)

    if CONST_SAVE_DATA == True:
        # Set the full scree mode.
        plt.get_current_fig_manager().full_screen_toggle()

        # Save the results.
        plt.savefig(f'{project_folder}/name.png', format='png', dpi=300)
    else:
        # Show the result.
        plt.show()

if __name__ == "__main__":
    sys.exit(main())