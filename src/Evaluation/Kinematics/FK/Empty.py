# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

def main():
    Robot_Str = Parameters.ABB_IRB_120_Str
    Robot_Str.T.Base = HTM_Cls(None, np.float32).Rotation(np.deg2rad([20.0, -20.0, 20.0]), 'ZYX')
    Robot_Str.T.Zero_Cfg = Kinematics.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

if __name__ == '__main__':
    sys.exit(main())