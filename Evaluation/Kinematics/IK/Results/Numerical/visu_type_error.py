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
CONST_NIK_METHOD = ['Newton-Raphson', 'Gauss-Newton', 'Levenberg-Marquardt']

def main():
    """
    Description:
        A program to compare the absolute/quadratic error in trajectory calculation using the numerical 
        method of inverse kinematics. 

        The comparison is tested on trajectories generated using a multi-axis polynomial profile.

        Note:
            The program for the generation of the absolute/quadratic errors
            can be found here:
                ../IK/collect_numerical_ik.py
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file was saved.
    file_path = f'{project_folder}/Data/Inverse_Kinematics/{Robot_Str.Name}'

    # Set the parameters for the scientific style.
    plt.style.use('science')

    data = []
    for _, nik_name in enumerate(CONST_NIK_METHOD):
        # Read data from the file.
        data.append(File_IO.Load(f'{file_path}/Method_Numerical_IK_{nik_name}_Error', 'txt', ','))

    label = [r'$e_{p}(t)$', r'$e_{q}(t)$', r'$E(t)$']; error_name = ['Absolute', 'Absolute', 'Quadratic']
    title = ['Absolute Position Error (APE)', 'Absolute Orientation Error (AOE)', 'Absolute Quadratic Error (AQE)']
    for i, label_i in enumerate(label):
        # Create a figure.
        _, ax = plt.subplots()

        e = []
        for j, data_i in enumerate(np.array(data, dtype=np.float64)):
            e.append(data_i[:, i])

            # Display the results as the values shown in the console.
            print(f'[INFO] Iteration: {j}')
            print(f'[INFO] Method: {CONST_NIK_METHOD[j]}')
            print(f'[INFO] Metrics: {label_i}')
            print(f'[INFO] max({label[i]}) = {np.max(data_i[:, i])}')
            print(f'[INFO] min({label[i]}) = {np.min(data_i[:, i])}')
            print(f'[INFO] M{error_name[i][0]}E = {np.mean(data_i[:, i])}')

        # Visualization of relevant structures.
        box_plot_out = ax.boxplot(e, labels=CONST_NIK_METHOD, showmeans=True, patch_artist=True, meanline = True, medianprops = dict(linestyle=None, linewidth=0.0),
                                  showfliers=False)
        
        # Set the properties of the box plot.
        #   Boxes.
        plt.setp(box_plot_out['boxes'][0], color='#e69138', facecolor='#f2c89b')
        plt.setp(box_plot_out['boxes'][1], color='#8ca8c5', facecolor='#c5d3e2')
        plt.setp(box_plot_out['boxes'][2], color='#a64d79', facecolor='#d2a6bc')
        #   Whiskers.
        plt.setp(box_plot_out['whiskers'][0], color='#e69138')
        plt.setp(box_plot_out['whiskers'][1], color='#e69138')
        plt.setp(box_plot_out['whiskers'][2], color='#8ca8c5')
        plt.setp(box_plot_out['whiskers'][3], color='#8ca8c5')
        plt.setp(box_plot_out['whiskers'][4], color='#a64d79')
        plt.setp(box_plot_out['whiskers'][5], color='#a64d79')
        #   Means.
        plt.setp(box_plot_out['means'][0], color='#e69138')
        plt.setp(box_plot_out['means'][1], color='#8ca8c5')
        plt.setp(box_plot_out['means'][2], color='#a64d79')
        #   Caps.
        plt.setp(box_plot_out['caps'][0], color='#e69138')
        plt.setp(box_plot_out['caps'][1], color='#e69138')
        plt.setp(box_plot_out['caps'][2], color='#8ca8c5')
        plt.setp(box_plot_out['caps'][3], color='#8ca8c5')
        plt.setp(box_plot_out['caps'][4], color='#a64d79')
        plt.setp(box_plot_out['caps'][5], color='#a64d79')

        # Set parameters of the graph (plot).
        ax.set_title(f'{title[i]}', fontsize=25, pad=25.0)
        #   Label
        ax.set_xlabel(r'Numerical Inverse Kinematics (IK) Method', fontsize=15, labelpad=10)
        ax.set_ylabel(f'{label[i]}', fontsize=15, labelpad=10) 
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
