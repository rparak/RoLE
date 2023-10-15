# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Custom Lib.:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Collider/Utilities
import Lib.Collider.Utilities
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Blender/Parameters/Camera
import Lib.Blender.Parameters.Camera

"""
Description:
    Open {robot_name}.blend from the URDFs folder and copy + paste this script and run it.

    Terminal:
        $ cd Documents/GitHub/Open_Industrial_Robotics/URDFs/Robots/{robot_name}/Blender
        $ blender {robot_name}.blend

    Note 1:
        Where the variable 'robot_name' is the name of the controlled robot to be used.

    Note 2:
        The program can also be used to generate colliders for primitive objects in the blender, see below:

        Terminal:
            $ cd Documents/GitHub/Open_Industrial_Robotics/Blender/Primitives/Primitives.blend
            $ blender Primitives.blend
"""

def main():
    """
    Description:
        A program to generate collision objects for individual robotic arms. The objects can be used for the 
        URDF files as collision geometry for the robot.

        The program will also generate code that will be used as an extension of the robot/mechanism structure 
        in the 'Parameters' folder.
        
            More information can be found in the programs below:
                ./Parameters/Robot.py or ./Parameters/Mechanism.py
    """

    
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()

    
    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()

    i = 0
    for _, obj in enumerate(bpy.data.objects):
        # Removes objects, if they exist.
        if 'Collision' in obj.name:
            Lib.Blender.Utilities.Remove_Object(obj.name)
            continue

        # Get positions of the vertices of the mesh object.
        obj_i_verts = Lib.Blender.Utilities.Get_Vertices_From_Object(obj.name)

        # Get the minimum and maximum X, Y, Z values of the input vertices.
        (min_vec3, max_vec3) = Lib.Collider.Utilities.Get_Min_Max(np.array(obj_i_verts, dtype=np.float64))

        # Obtain the size and centroid of the observed object.
        Size = np.array([max_vec3[0] - min_vec3[0], max_vec3[1] - min_vec3[1],
                         max_vec3[2] - min_vec3[2]], dtype=np.float64)
        Centroid = np.array([(max_vec3[0] + min_vec3[0]) / 2.0, (max_vec3[1] + min_vec3[1]) / 2.0,
                             (max_vec3[2] + min_vec3[2]) / 2.0], dtype=np.float64)
        
        # Create a new name for the collision object.
        obj_i_name_new = obj.name.removesuffix('Visual') + 'Collision'

        # Properties of the created object.
        box_properties = {'transformation': {'Size': 1.0, 
                                             'Scale': Size, 
                                             'Location': Centroid}, 
                          'material': {'RGBA': [1.0,1.0,1.0,1.0], 'alpha': 1.0}}
        
        # Create a primitive three-dimensional object (cuboid) with additional properties.
        Lib.Blender.Utilities.Create_Primitive('Cube', obj_i_name_new, box_properties)
        
        # Set the rotation mode to be the same as the observed object.
        bpy.data.objects[obj_i_name_new].rotation_mode = bpy.data.objects[obj.name].rotation_mode
        
        # Set the origin of the observed object to zero.
        Lib.Blender.Utilities.Set_Object_Origin(obj_i_name_new, HTM_Cls(None, np.float64).all())


        # Remove all materials from the created object.
        Lib.Blender.Utilities.Remove_Object_Material(obj_i_name_new)

        # Origin (o), Size (s)
        o = np.round((-1) * Centroid, 5) + [0.0, 0.0, 0.0]; s = np.round(Size, 5) + [0.0, 0.0, 0.0]
        
        # Display results.
        print(f'[INFO] Object name: {obj.name}')
        print(f'[INFO] >> OBB_Cls(Box_Cls([{o[0]:.05f}, {o[1]:.05f}, {o[2]:.05f}], [{s[0]:.05f}, {s[1]:.05f}, {s[2]:.05f}]))')
        i += 1

if __name__ == '__main__':
    main()