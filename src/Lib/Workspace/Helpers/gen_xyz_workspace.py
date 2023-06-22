# Platform (Platform identification data)
import platform
# System (Default)
import sys
if platform.system() == 'Windows':
    # Windows Path.
    sys.path.append('..\\..\\..\\..')
else:
    # Linux (Ubuntu) Path.
    sys.path.append('../../../..' + 'src') 
# Custom Script:
#   ../Lib/Manipulator/Parameters (The main parameters of the manipulator)
import Lib.Manipulator.Parameters as Parameters
#   ../Lib/Manipulator/Workspace/Core
import Lib.Manipulator.Workspace.Core

def main():
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = Parameters.ABB_IRB_120_Structure

    # Input File: Absolute orientation of the joints.
    CONST_FILE_NAME_IN = 'abs_joint_orient_data'
    print(f'[INFO] Path (Input File): ../../../../../Data/Workspace/{Robot_Str.Name}{CONST_FILE_NAME_IN}.txt')
    # Output File: X, Y, Z positions of the workspace.
    CONST_FILE_NAME_OUT = 'tool0_workspace_data'
    print(f'[INFO] Path (Output File): ../../../../../Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_OUT}.txt')

    # Generate x, y, z positions of the workspace from the absolute orientation 
    # of the joints.
    Lib.Manipulator.Workspace.Core.Generate_Workspace_XYZ(Robot_Str, f'../../../../../Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_IN}.txt', 
                                                          f'../../../../../Data/Workspace/{Robot_Str.Name}/{CONST_FILE_NAME_OUT}.txt')
    
if __name__ == "__main__":
    sys.exit(main())