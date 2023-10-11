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

    # Read data from the file.
    data = File_IO.Load(f'{file_path}/Method_Numerical_IK_{CONST_NIK_METHOD}_TCP', 'txt', ',')

    for i, data_i in enumerate(data):
        # Create a figure.
        _, ax = plt.subplots()

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