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
CONST_SAVE_DATA = True

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

    # ...
    t = np.arange(0.0, len(data[:, 0]), 1)

    # ...
    marker = ['o', 'x', '.']; label = [r'$e_{p}$', r'$e_{q}$', r'E']; error_name = ['Absolute', 'Absolute', 'Quadratic']
    for i, data_i in enumerate(data.T):
        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        ax.plot(t, data_i, marker[i], color='#8d8d8d', linewidth=3.0, markersize=8.0, markeredgewidth=3.0, markerfacecolor='#8d8d8d', label=label[i])
        ax.plot(t, [np.mean(data_i)] * t.size, '--', color='#8d8d8d', linewidth=1.5, label=f'Mean {error_name[i]} Error')

        # Set parameters of the graph (plot).
        #   Set the x ticks.
        ax.set_xticks(np.arange(np.min(t) - 10, np.max(t) + 10, 10))
        #   Label
        ax.set_xlabel(r'Number of TCP (Tool Center Point) targets', fontsize=15, labelpad=10)
        ax.set_ylabel(f'{error_name[i]} error {label[i]} in millimeters', fontsize=15, labelpad=10) 
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
            if CONST_IK_METHOD == 'Analytical':
                plt.savefig(f'{project_folder}/images/IK/{Robot_Str.Name}/Method_{CONST_IK_METHOD}_IK_Error_{label[i]}.png', 
                            format='png', dpi=300)
            elif CONST_IK_METHOD == 'Numerical':
                plt.savefig(f'{project_folder}/images/IK/{Robot_Str.Name}/Method_{CONST_IK_METHOD}_IK_{CONST_NIK_METHOD}_Error_{label[i]}.png', 
                            format='png', dpi=300)
        else:
            # Show the result.
            plt.show()

if __name__ == "__main__":
    sys.exit(main())