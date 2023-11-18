# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
import pybullet_data
# Time (Time access and conversions)
import time
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('RoLE')[0] + 'RoLE'

def main():
    """
    Description:
        Program to verify the correctness of the generated URDF file for the robotic arm 
        in the PyBullet environment.

        The project, which explains how to use a PyBullet physics simulation 
        environment, can be found here:
            https://github.com/rparak/PyBullet_Template_Industrial_Robotics/
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Connect to the physics simulation and create an environment with additional properties.
    pb.connect(pb.GUI)
    pb.setTimeStep(0.01)
    pb.setRealTimeSimulation(0)
    pb.resetSimulation()
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0.0, 0.0, -9.81)

    # Load a physics model of the robotic structure.
    _ = pb.loadURDF(f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', [0.0, 0.0, 0.0], 
                    [0.0, 0.0, 0.0, 1.0], useFixedBase=True, flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
    
    while pb.isConnected():
        pb.stepSimulation()

        # The time to approximate and update the state of the dynamic system.
        time.sleep(0.01)
    
    # A function to disconnect the created environment from a physical server.
    pb.disconnect()

if __name__ == '__main__':
    main()