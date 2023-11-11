# Typing (Support for type hints)
import typing as tp

def Get_Physical_Properties(name: str) -> tp.Tuple[float]:
    """
    Description:
        Get the defined physical properties of the robotic structure.

            Properties:
                Mass:
                    Weight of individual robot parts in kilograms.
                Effort:
                    An attribute for enforcing the maximum joint effort.
                Velocity:
                    An attribute for enforcing the maximum joint velocity.

          Warning:
            Please note that these values are approximate.
                
    Args:
        (1) name [string]: Name of the robotic structure.

    Returns:
        (1) parameter [Dictionary {'mass', Vector<float> 1xn, etc.}]: Defined physical properties of the robotic 
                                                                      structure.
                                                                        Note:
                                                                            Where n is the number of joints + base.
    """

    return {
        'Universal_Robots_UR3': {'mass': [2.0, 2.0, 3.42, 1.26, 0.8, 0.8, 0.35], 
                                 'effort': [56.0, 56.0, 28.0, 12.0, 12.0, 12.0], 
                                 'velocity': [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]},
        'ABB_IRB_120': {'mass': [8.25, 3.45, 5.9, 2.95, 2.30, 1.55, 0.60], 
                        'effort': [88.0, 88.0, 44.0, 22.0, 22.0, 22.0], 
                        'velocity': [4.36, 4.36, 4.36, 5.58, 5.58, 7.33]},
        'ABB_IRB_120_L_Ax': {'mass': [46.0, 4.0, 8.25, 3.45, 5.9, 2.95, 2.30, 1.55, 0.60], 
                             'effort': [4.0, 2.0, 2.0, 1.0, 0.5, 0.5, 0.5], 
                             'velocity': [0.5, 4.36, 4.36, 4.36, 5.58, 5.58, 7.33]},
        'ABB_IRB_14000_R': {'mass': [0.0, 2.0, 2.0, 2.5, 2.0, 2.0, 2.0, 0.5],
                            'effort': [36.0, 36.0, 12.0, 12.0, 6.0, 6.0, 6.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'ABB_IRB_14000_L': {'mass': [0.0, 2.0, 2.0, 2.5, 2.0, 2.0, 2.0, 0.5], 
                            'effort': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'EPSON_LS3_B401S': {'mass': [5.5, 3.0, 4.5, 0.5, 0.5], 
                            'effort': [88.0, 88.0, 44.0, 22.0], 
                            'velocity': [6.28, 6.28, 1.0, 12.56]}
    }[name]