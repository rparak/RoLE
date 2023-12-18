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
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file will be saved.
    file_path = f'{project_folder}/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Read data from the file.
    data = File_IO.Load(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Desired', 'txt', ',')
    data_predicted = File_IO.Load(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP_Predicted', 'txt', ',')

    # Get the normalized time.
    t_hat = np.linspace(0.0, 1.0, len(data[:, 0]))

    # Display TCP(Tool Center Point) parameters.
    y_label = [r'$x(\hat{t})$ in meters', r'$y(\hat{t})$ in meters', r'$z(\hat{t})$ in meters', r'$q_{w}(\hat{t})$ in [-]', 
               r'$q_{x}(\hat{t})$ in [-]', r'$q_{y}(\hat{t})$ in [-]', r'$q_{z}(\hat{t})$ in [-]']
    for i, (data_i, data_p_i) in enumerate(zip(data.T, data_predicted.T)):
        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        ax.plot(t_hat, data_i, '.-', color='#d0d0d0', linewidth=1.0, markersize = 3.0, 
                markeredgewidth = 1.5, markerfacecolor = '#ffffff', label=f'Desired data')
        ax.plot(t_hat, data_p_i, '.-', color='#ffbf80', linewidth=1.0, markersize = 3.0, 
                markeredgewidth = 1.5, markerfacecolor = '#ffffff', label=f'Numerical {CONST_NIK_METHOD} Method: Predicted data')

        # Set parameters of the graph (plot).
        #   Set the x ticks.
        ax.set_xticks(np.arange(np.min(t_hat) - 0.1, np.max(t_hat) + 0.1, 0.1))
        #   Set the y ticks.
        tick_y_tmp = (np.max(data_i) - np.min(data_i))/10.0
        tick_y = tick_y_tmp if tick_y_tmp != 0.0 else 0.1
        ax.set_yticks(np.arange(np.min(data_i) - tick_y, np.max(data_i) + tick_y, tick_y))
        #   Label.
        ax.set_xlabel(r'Normalized time $\hat{t}$ in the range of [0.0, 1.0]', fontsize=15, labelpad=10)
        ax.set_ylabel(f'{y_label[i]}', fontsize=15, labelpad=10) 
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
            plt.savefig(f'{project_folder}/images/IK/{Robot_Str.Name}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP.png', 
                        format='png', dpi=300)
        else:
            # Show the result.
            plt.show()

if __name__ == "__main__":
    sys.exit(main())