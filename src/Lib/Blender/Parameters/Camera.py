# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Custom Library:
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

@dataclass
class Camera_Parameters_Str:
    """
    Description:
        The structure of the main parameters of the camera.

    Example:
        Initialization:
            Cls = Camera_Parameters_Str()
            Cls.T = ...
            ...
            Cls.Value = ..

    """

    # Transformation matrix of the object.
    #   Unit [Matrix<float>]
    T: tp.List[tp.List[float]] = field(default_factory=list)
    #  The properties of the projection view.
    #   Unit [1: string, 2: float]
    #   1\ Projection of the camera's field of view: Perspective = ['PERSP'], Orthographic = ['ORTHO']
    Type: str = ''
    #   2\ Value is a PERSPECTIVE CAMERA LENS for perspective view, and ORTHOGRAPHIC CAMERA SCALE for ortographic view.
    Value: float = 0.0

"""
Camera view from the right.

    The properties of the projection view: Perspective, 50.0
"""
Right_View_Camera_Parameters_Str = Camera_Parameters_Str()
Right_View_Camera_Parameters_Str.T = HTM_Cls([[ -0.4226, -0.3099, 0.8516, 3.2500],
                                              [  0.9063, -0.1445, 0.3971, 1.5000],
                                              [  0.0000,  0.9396, 0.3420, 1.4500],
                                              [  0.0000,  0.0000, 0.0000, 1.0000]], np.float32)
Right_View_Camera_Parameters_Str.Type  = 'PERSP'
Right_View_Camera_Parameters_Str.Value = 50.0

