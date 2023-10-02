import numpy as np
import Lib.Manipulator.Kinematics.Core as Kinematics
import Lib.Manipulator.Parameters as Parameters
import Lib.Manipulator.Utilities.Mathematics as Mathematics
import Lib.Manipulator.Utilities.Transformation 
import Lib.Manipulator.Utilities.General as General

# work, work, work!!!
# https://studywolf.wordpress.com/tag/quaternions/
# http://www.osrobotics.org/osr/kinematics/forward_kinematics.html

# add collision detection using box? Or not yet?
# Input data:
q = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 

T_current = Kinematics.Forward_Kinematics(q, 'Modified', Parameters.Universal_Robots_UR3_Structure)[1]
#print(f"The manipulator FK is: \n{np.round(T_current, 3)}")

#T_desired_R = Transformation.Get_Matrix_From_Euler_Method_Fast(np.deg2rad([90.0, 0.0, -180.0]), 'ZYX')
#T_desired = Transformation.Set_Matrix_Translation(T_desired_R, [0.2, -0.104, 0.4])

T_Identity = Lib.Manipulator.Utilities.Transformation.Get_Matrix_Identity(4)
T_desired  = Lib.Manipulator.Utilities.Transformation.Homogeneous_Transformation_Matrix_Cls(T_Identity, np.float64).Rotation([1.57079633, 0.0, -3.14159265], 'ZYX').Translation([0.2, -0.104, 0.4])

# Error
e = Mathematics.Euclidean_Norm((T_current - T_desired).all())

#print(np.round(T_current.all(), 4))
#print(np.round(T_desired.all(), 4))

#print(T_desired.p)
#print(T_desired.Scale([1.0,1.0,1.0]))
#print(T_desired.Get_Scale_Vector())


W_e = np.diag(np.ones(6))
iteration = 0; q_bef = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
while True:
    #print(f'Joint angles in iteration {i}: {np.rad2deg(q)}')
    #print(f'Actual error: {e}')
    #print(f'Joint angles (result): {np.round(np.rad2deg(q), 2)}')
    J = Kinematics.Get_Geometric_Jacobian(q, Parameters.Universal_Robots_UR3_Structure)

    """
    if np.linalg.det(J) != 0:
        q += np.linalg.inv(J) @ General.Get_Angle_Axis_Error(T_desired, T_current)
    else:
        q += np.linalg.pinv(J) @ General.Get_Angle_Axis_Error(T_desired, T_current)
    """
    # Maybe?
    q += np.linalg.pinv(J) @ General.Get_Angle_Axis_Error(T_desired, T_current) 

    q_limit_err, T_current = Kinematics.Forward_Kinematics(q, 'Modified', Parameters.Universal_Robots_UR3_Structure)

    e = Mathematics.Euclidean_Norm((T_current - T_desired).all())
    
    if e < 0.0000001:
        break

    for i, q_limit_err_i in enumerate(q_limit_err):
        if q_limit_err_i == True:
            q[i] = q_bef[i]
        else:
            q_bef[i] = q[i]

    iteration += 1

#print(q_limit_err.all() == False)
print(f'Joint angles (result): {np.round(np.rad2deg(q), 2)}')
print(f'Error: {e}')
print(f'Done in iteration {iteration}!')
_, T_current_Final = Kinematics.Forward_Kinematics(q, 'Modified', Parameters.Universal_Robots_UR3_Structure)

e = Mathematics.Euclidean_Norm((T_current_Final - T_desired).all())
print(f'Error: {e}')