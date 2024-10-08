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

def main():
    """
    Description:
        A program to visualize the absolute error in trajectory calculation using the analytical 
        method of inverse kinematics.

        The observation is tested on trajectories generated using a multi-axis trapezoidal profile.

        Note:
            The program for the generation of the absolute/quadratic errors
            can be found here:
                ../IK/collect_analytical_ik.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file was saved.
    file_path = f'{project_folder}/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Read data from the file.
    data = File_IO.Load(f'{file_path}/Method_Analytical_IK_Error', 'txt', ',')
    
    # Get the normalized time.
    t_hat = np.linspace(0.0, 1.0, len(data[:, 0]))

    label = [r'$e_{p}(\hat{t})$', r'$e_{q}(\hat{t})$']; title = ['Absolute Position Error (APE)', 
                                                                 'Absolute Orientation Error (AOE)']
    for i, data_i in enumerate(data.T):
        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        ax.plot(t_hat, [np.mean(data_i)] * t_hat.size, '--', color='#bababa', linewidth=1.0, label=f'Mean Absolute Error (MAE)')
        ax.plot(t_hat, data_i, '.', color='#8d8d8d', alpha=1.0, markersize=8.0, markeredgewidth=2.0, markerfacecolor='#ffffff', label=label[i])

        # Set parameters of the graph (plot).
        ax.set_title(f'{title[i]}', fontsize=25, pad=25.0)
        #   Set the x ticks.
        ax.set_xticks(np.arange(np.min(t_hat) - 0.1, np.max(t_hat) + 0.1, 0.1))
        #   Label
        ax.set_xlabel(r'Normalized time $\hat{t}$ in the range of [0.0, 1.0]', fontsize=15, labelpad=10)
        ax.set_ylabel(f'{label[i]}', fontsize=15, labelpad=10) 
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
        print(f'[INFO] max(label{i}) = {np.max(data_i)} in meters')
        print(f'[INFO] min(label{i}) = {np.min(data_i)} in meters')
        print(f'[INFO] MAE = {np.mean(data_i)} in mm')

        # Show the result.
        plt.show()
            
if __name__ == "__main__":
    sys.exit(main())
