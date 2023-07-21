# Typing (Support for type hints)
import typing as tp

def Get_Physical_Properties(name: str) -> tp.Dict[float]:
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
                
    Args:
        (1) name [string]: Name of the robotic structure.

    Returns:
        (1) parameter [Dictionary {'mass', Vector<float> 1xn, etc.}]: Defined physical properties of the robotic 
                                                                      structure.
    """

    return {
        'Universal_Robots_UR3': {'mass': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                                 'effort': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                                 'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'ABB_IRB_120': {'mass': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                        'effort': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                        'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'ABB_IRB_120_L_Ax': {'mass': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                             'effort': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                             'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'ABB_IRB_14000_R': {'mass': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                            'effort': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'ABB_IRB_14000_L': {'mass': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                            'effort': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'EPSON_LS3_B401S': {'mass': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                            'effort': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}
    }[name]