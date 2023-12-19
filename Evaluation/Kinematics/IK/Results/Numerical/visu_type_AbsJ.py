# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# SciencePlots (Matplotlib styles for scientific plotting) [pip3 install SciencePlots]
import scienceplots
# Matplotlib (Visualization) [pip3 install matplotlib]
import matplotlib.pyplot as plt
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO as File_IO

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.EPSON_LS3_B401S_Str
# Name of the numerical methods used to calculate the IK solution.
CONST_NIK_METHOD = ['Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton',
                    'Levenberg-Marquardt']
# Save the data to a file.
CONST_SAVE_DATA = False

def main():
    """
    Description:
        ...
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file will be saved.
    file_path = f'{project_folder}/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Read data from the file.
    data = []
    for _, nik_name in enumerate(CONST_NIK_METHOD):
        data.append(File_IO.Load(f'{file_path}/Method_Numerical_IK_{nik_name}_Absolute_Joint_Positions', 'txt', ','))

    # Get the normalized time.
    t_hat = np.linspace(0.0, 1.0, len(data[0][:, 0]))

    # Display absolute joint position parameters.
    for i in range(Robot_Str.Theta.Home.size):
        # Create a figure.
        _, ax = plt.subplots()

        for j, (data_i, c_i) in enumerate(zip(np.array(data, dtype=np.float64),
                                              ['#a64d79', '#dbdbbf', '#bfdbd1', '#abcae4'])):
            # Visualization of relevant structures.
            ax.plot(t_hat, data_i[:, i], '--', color=c_i, linewidth=1.0, markerfacecolor=c_i, 
                    label=f'{CONST_NIK_METHOD[j]} Method')

            # Set parameters of the graph (plot).
            #   Set the x ticks.
            ax.set_xticks(np.arange(np.min(t_hat) - 0.1, np.max(t_hat) + 0.1, 0.1))
            #   Set the y ticks.
            tick_y_tmp = (np.max(data_i[:, i]) - np.min( data_i[:, i]))/10.0
            tick_y = tick_y_tmp if tick_y_tmp != 0.0 else 0.1
            ax.set_yticks(np.arange(np.min( data_i[:, i]) - tick_y, np.max( data_i[:, i]) + tick_y, tick_y))
            #   Label.
            ax.set_xlabel(r'Normalized time $\hat{t}$ in the range of [0.0, 1.0]', fontsize=15, labelpad=10)
            ax.set_ylabel(r'$\theta_{%d}(t)$ in %s' % ((i + 1), 'radians' if Robot_Str.Theta.Type[i] == 'R' else 'meters'), 
                        fontsize=15, labelpad=10) 
            #   Set parameters of the visualization.
            ax.grid(which='major', linewidth = 0.15, linestyle = '--')
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