# BPY (Blender as a python) [pip3 install bpy]
import bpy
# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Collider/Utilities
import Lib.Collider.Utilities
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../Lib/Primitives/Core
from Lib.Primitives.Core import Box_Cls
#   ../Lib.Collider.Core
import Lib.Collider.Core 

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

def main():
    # Deselect all objects in the current scene.
    Lib.Blender.Utilities.Deselect_All()

    # Remove animation data from objects (Clear keyframes).
    Lib.Blender.Utilities.Remove_Animation_Data()


    for obj in bpy.data.objects:
        # Removes objects, if they exist.
        if 'Collision' in obj.name:
            Lib.Blender.Utilities.Remove_Object(obj.name)
            continue

        if obj.name == 'Base_Visual':
            # ...
            obj_i_name_new = obj.name.removesuffix('Visual') + 'Collision'

            # ...
            obj_i_verts = Lib.Blender.Utilities.Get_Vertices_From_Object(obj.name)

            # ...
            (min_vec3, max_vec3) = Lib.Collider.Utilities.Get_Min_Max(np.array(obj_i_verts, dtype=np.float32))

            Size = np.array([max_vec3[0] - min_vec3[0], max_vec3[1] - min_vec3[1],
                    max_vec3[2] - min_vec3[2]], dtype=np.float32)
            Centroid = np.array([(max_vec3[0] + min_vec3[0]) / 2.0, (max_vec3[1] + min_vec3[1]) / 2.0,
                        (max_vec3[2] + min_vec3[2]) / 2.0], dtype=np.float32)
            # Properties of the created object.
            box_properties = {'transformation': {'Size': 1.0, 
                                                 'Scale': Size, 
                                                 'Location': Centroid}, 
                            'material': {'RGBA': [1.0,1.0,1.0,1.0], 'alpha': 1.0}}
            
            # Create a primitive three-dimensional object (cuboid) with additional properties.
            Lib.Blender.Utilities.Create_Primitive('Cube', obj_i_name_new, box_properties)
            
            # ...
            bpy.data.objects[obj_i_name_new].rotation_mode = bpy.data.objects[obj.name].rotation_mode
            
            # ...
            Lib.Blender.Utilities.Set_Object_Origin(obj_i_name_new, HTM_Cls(None, np.float32).all())

            # ...
            Box_i = Box_Cls((-1) * Centroid, Size)
            # ...
            OBB_i = Lib.Collider.Core.OBB_Cls(Box_i)
            OBB_i.Transformation(HTM_Cls(None, np.float32))

if __name__ == '__main__':
    main()