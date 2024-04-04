# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics

def Get_Absolute_Joint_Positions(name: str) -> tp.Tuple[tp.List[float],
                                                        tp.List[float]]:
    """
    Description:
        A function to obtain the constraints for absolute joint positions in order to generate 
        multi-axis position trajectories.

    Args:
        (1) name [string]: Name of the robotic structure.

    Returns:
        (1) parameter [Vector<float> 2xn]: Obtained absolute joint positions (initial, final) in radians / meters.
                                            Note:
                                                Where n is the number of joints.
    """

    return {
        'Universal_Robots_UR3': (Mathematics.Degree_To_Radian(np.array([-110.0, -55.0, -85.0, -105.0, 80.0, 45.0], 
                                                                       dtype=np.float64)),
                                 Mathematics.Degree_To_Radian(np.array([-60.0, -95.0, -115.0, -10.0, 40.0, -45.0], 
                                                                       dtype=np.float64))),
        'ABB_IRB_120': (Mathematics.Degree_To_Radian(np.array([-60.0, -20.0, 25.0, 40.0, 50.0, -45.0], 
                                                              dtype=np.float64)),
                        Mathematics.Degree_To_Radian(np.array([40.0, 25.0, 15.0, -45.0, 60.0, 15.0], 
                                                              dtype=np.float64))),
        'ABB_IRB_120_L_Ax': (np.append([0.0], Mathematics.Degree_To_Radian(np.array([-10.0, -20.0, 25.0, 40.0, 50.0, -45.0], 
                                                                                    dtype=np.float64))), 
                             np.append([0.4], Mathematics.Degree_To_Radian(np.array([40.0, 25.0, 15.0, -45.0, 60.0, 15.0], 
                                                                                    dtype=np.float64)))),
        'ABB_IRB_14000_R': (Mathematics.Degree_To_Radian(np.array([15.0, -115.0, -90.0, 10.0, 15.0, 65.0, -115.0], 
                                                                  dtype=np.float64)), 
                            Mathematics.Degree_To_Radian(np.array([65.0, -40.0, -125.0, 15.0, 50.0, 90.0, -60.0], 
                                                                  dtype=np.float64))),
        'ABB_IRB_14000_L': (Mathematics.Degree_To_Radian(np.array([-15.0, -115.0, 90.0, 10.0, 15.0, 65.0, -115.0], 
                                                                  dtype=np.float64)), 
                            Mathematics.Degree_To_Radian(np.array([-65.0, -40.0, 125.0, 15.0, -50.0, 90.0, -60.0], 
                                                                  dtype=np.float64))),
        'EPSON_LS3_B401S': (np.array([Mathematics.Degree_To_Radian(-40.0), Mathematics.Degree_To_Radian(50.0), 0.0, Mathematics.Degree_To_Radian(-30.0)], 
                                     dtype = np.float64), 
                            np.array([Mathematics.Degree_To_Radian(115.0), Mathematics.Degree_To_Radian(-20.0), 0.10, Mathematics.Degree_To_Radian(15.0)],
                                     dtype = np.float64))
    }[name]