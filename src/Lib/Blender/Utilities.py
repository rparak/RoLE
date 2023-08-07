# BPY (Blender as a python) [pip3 install bpy]
import bpy
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]tp
import numpy as np
# OS (Operating system interfaces)
import os
# BMesh (Access to blenders bmesh data)
import bmesh
# Mathutils (Math Types & Utilities)
import mathutils 
# Custom Library:
#   ../Lib/Transformation/Core
import Lib.Transformation.Core as Transformation
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot
#   ../Lib/Blender/Core
import Lib.Blender.Core
#   ../Lib/Blender/Parameters/Camera
import Lib.Blender.Parameters.Camera


def Deselect_All() -> None:
    """
    Description:
        Deselect all objects in the current scene.
    """
    
    for obj in bpy.context.selected_objects:
        bpy.data.objects[obj.name].select_set(False)
    bpy.context.view_layer.update()
        
def Object_Exist(name: str) -> bool:
    """
    Description:
        Check if the object exists within the scene.
        
    Args:
        (1) name [string]: Object name.
        
    Returns:
        (1) parameter [bool]: 'True' if it exists, otherwise 'False'.
    """
    
    return True if bpy.context.scene.objects.get(name) else False

def Remove_Object(name: str) -> None:
    """
    Description:
        Remove the object (hierarchy) from the scene, if it exists. 

    Args:
        (1) name [string]: The name of the object.
    """

    # Find the object with the desired name in the scene.
    object_name = None
    for obj in bpy.data.objects:
        if name in obj.name and Object_Exist(obj.name) == True:
            object_name = obj.name
            break

    # If the object exists, remove it, as well as the other objects in the hierarchy.
    if object_name is not None:
        bpy.data.objects[object_name].select_set(True)
        for child in bpy.data.objects[object_name].children:
            child.select_set(True)
        bpy.ops.object.delete()
        bpy.context.view_layer.update()

def Remove_Object_Material(name: str) -> None:
    """
    Description:
        Remove all materials from the individual object.
            
    Args:
        (1) name [string]: The name of the object.
    """
        
    # Create a default material. Due to a possible change in the color of the object. 
    material_obj = bpy.data.materials.new('Default_Material')
    material_obj.use_nodes = True
    material_obj.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = (0.75,0.75,0.75,1.0) 
        
    for obj in bpy.data.objects:
        if obj == bpy.data.objects[name]:
            # Remove all materials (material slots) from the object.
            obj.active_material_index = 0
            for _ in range(len(obj.material_slots)):
                bpy.ops.object.material_slot_remove({'object': obj})

            # Assign material to the active object.
            bpy.context.view_layer.objects.active = bpy.data.objects[name]
            bpy.context.active_object.active_material = material_obj
            bpy.context.view_layer.objects.active = None
    
    # Deselect all objects in the current scene.
    Deselect_All()

def Set_Object_Material_Color(name: str, color: tp.List[float]):
    """
    Description:
        Set the material color of the individual object and/or the object hierarchy (if exists).
            
    Args:
        (1) name [string]: The name of the object.
        (2) color [Vector<float>]: RGBA color values: rgba(red, green, blue, alpha).
    """

    for obj in bpy.data.objects:
        if bpy.data.objects[name].parent == True:
            if obj.parent == bpy.data.objects[name]:
                for material in obj.material_slots:
                    material.material.node_tree.nodes['Principled BSDF'].inputs["Base Color"].default_value = color

                 # Recursive call.
                return Set_Object_Material_Color(obj.name, color)
        else:
            if obj == bpy.data.objects[name]:
                for material in obj.material_slots:
                    material.material.node_tree.nodes['Principled BSDF'].inputs["Base Color"].default_value = color 

def Set_Object_Material_Transparency(name: str, alpha: float) -> None:
    """
    Description:
        Set the transparency of the object material and/or the object hierarchy (if exists).
        
        Note: 
            alpha = 1.0: Render surface without transparency.
            
    Args:
        (1) name [string]: The name of the object.
        (2) alpha [float]: Transparency information.
                           (total transparency is 0.0 and total opacity is 1.0)
    """

    for obj in bpy.data.objects:
        if bpy.data.objects[name].parent == True:
            if obj.parent == bpy.data.objects[name]:
                for material in obj.material_slots:
                    if alpha == 1.0:
                        material.material.blend_method  = 'OPAQUE'
                    else:
                        material.material.blend_method  = 'BLEND'
                    
                    material.material.shadow_method = 'OPAQUE'
                    material.material.node_tree.nodes['Principled BSDF'].inputs['Alpha'].default_value = alpha
                
                # Recursive call.
                return Set_Object_Material_Transparency(obj.name, alpha)
        else:
            if obj == bpy.data.objects[name]:
                for material in obj.material_slots:
                    if alpha == 1.0:
                        material.material.blend_method  = 'OPAQUE'
                    else:
                        material.material.blend_method  = 'BLEND'
                    
                    material.material.shadow_method = 'OPAQUE'
                    material.material.node_tree.nodes['Principled BSDF'].inputs['Alpha'].default_value = alpha

def __Add_Primitive(type: str, properties: tp.Tuple[float, tp.List[float], tp.List[float]]) -> bpy.ops.mesh:
    """
    Description:
        Add a primitive three-dimensional object.
        
    Args:
        (1) type [string]: Type of the object. 
                            Primitives: ['Plane', 'Cube', 'Sphere', 'Capsule']
        (2) properties [Dictionary {'Size/Radius': float, 'Scale/Size/None': Vector<float>, 
                                    'Location': Vector<float>]: Transformation properties of the created object. The structure depends 
                                                                on the specific object.
    
    Returns:
        (1) parameter [bpy.ops.mesh]: Individual three-dimensional object (primitive).
    """
        
    return {
        'Plane': lambda x: bpy.ops.mesh.primitive_plane_add(size=x['Size'], scale=x['Scale'], location=x['Location']),
        'Cube': lambda x: bpy.ops.mesh.primitive_cube_add(size=x['Size'], scale=x['Scale'], location=x['Location']),
        'Sphere': lambda x: bpy.ops.mesh.primitive_uv_sphere_add(radius=x['Radius'], location=x['Location']),
        'Capsule': lambda x: bpy.ops.mesh.primitive_round_cube_add(radius=x['Radius'], size=x['Size'], location=x['Location'], arc_div=10)
    }[type](properties)

def Create_Primitive(type: str, name: str, properties: tp.Tuple[tp.Tuple[float, tp.List[float]], tp.Tuple[float]]) -> None:
    """
    Description:
        Create a primitive three-dimensional object with additional properties.

    Args:
        (1) type [string]: Type of the object. 
                            Primitives: ['Plane', 'Cube', 'Sphere', 'Capsule']
        (2) name [string]: The name of the created object.
        (3) properties [{'transformation': {'Size/Radius': float, 'Scale/Size/None': Vector<float>, 'Location': Vector<float>}, 
                         'material': {'RGBA': Vector<float>, 'alpha': float}}]: Properties of the created object. The structure depends on 
                                                                                on the specific object.
    """

    # Create a new material and set the material color of the object.
    material = bpy.data.materials.new(f'{name}_mat')
    material.use_nodes = True
    material.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = properties['material']['RGBA']

    # Add a primitive three-dimensional object.
    __Add_Primitive(type, properties['transformation'])

    # Change the name and material of the object.
    bpy.context.active_object.name = name
    bpy.context.active_object.active_material = material

    # Set the transparency of the object material.
    if properties['material']['alpha'] < 1.0:
        Set_Object_Material_Transparency(name, properties['material']['alpha'])

    # Deselect all objects in the current scene.
    Deselect_All()

    # Update the scene.
    bpy.context.view_layer.update()

def Get_Object_Hierarchy(name: str) -> None:
    """
    Description: 
        Get the hierarchy of objects from the object structure.
        
    Args:
        (1) name [string]: Name of the main object.

    Example:
        Get_Object_Hierarchy(bpy.data.objects['ABB_IRB_120'])
    """
    
    for obj in bpy.data.objects:
        if obj.name == bpy.data.objects[name]:
            # Print name of the current object (index i).
            print(obj.name)
            # Recursive call.
            return Get_Object_Hierarchy(obj.name)

def Remove_Animation_Data() -> None:
    """
    Description: 
        Remove animation data from objects (Clear keyframes).
    """
    
    for obj in bpy.data.objects:
        obj.animation_data_clear()

def Object_Visibility(name: str, state: bool) -> None:
    """
    Description:
        Function to enable and disable the visibility of an object.
    
    Args:
        (1) name [string]: Name of the main object.
        (2) state [bool]: Enable (True) / Disable (False).  
    """
    
    cmd = not state; obj = bpy.data.objects[name]
    
    if Object_Exist(name):
        obj.hide_viewport = cmd; obj.hide_render = cmd
        for obj_i in obj.children:
            obj_i.hide_viewport = cmd; obj_i.hide_render = cmd

def Set_Object_Transformation(name: str, T: tp.List[tp.List[float]]) -> None:
    """
    Description:
        Set the object transformation.
        
    Args:
        (1) name [string]: Name of the main object.
        (2) T [Matrix<float> 4x4]: Homogeneous transformation matrix (access to location, rotation and scale).
    """

    if isinstance(T, (list, np.ndarray)):
        T = Transformation.Homogeneous_Transformation_Matrix_Cls(T, np.float32)
    
    bpy.data.objects[name].matrix_basis = T.Transpose().all()

def Duplicate_Object(name: str, identification: str):
    """
    Description:
        Duplication of objects and/or object hierarchy (if exists).
        
    Args:
        (1) name [string]: Name of the main object.
        (2) identification [string]: Identification of the name of the new object.
                                     (name + '_' + identification)
    """
    
    # Select the main object and/or object hierarchy.
    bpy.data.objects[name].select_set(True)
    if bpy.data.objects[name].children:
        # If the object has children(s).
        for obj in bpy.data.objects[name].children:        
            bpy.data.objects[obj.name].select_set(True)
    # Duplicate the selected object(s).
    bpy.ops.object.duplicate()
    
    # Deselect the current object / object hierarchy.
    for obj in bpy.context.selected_objects:
        if name in obj.name:
            # Rename the main object.
            obj.name = name + '_' + identification
        bpy.data.objects[obj.name].select_set(False)
    
    bpy.context.view_layer.update()

def Set_Camera_Properties(name: str, Camera_Parameters_Str: Lib.Blender.Parameters.Camera.Camera_Parameters_Str):
    """
    Description:
        Set the camera (object) transformation and projection.

    Args:
        (1) name [string]: Object name.
        (2) Camera_Parameters_Str [Camera_Parameters_Str(object)]: The structure of the main parameters of the camera.
    """

    # Set the object transformation.
    Set_Object_Transformation(name, Camera_Parameters_Str.T)

    # Set the projection of the camera.
    bpy.data.cameras[name].type = Camera_Parameters_Str.Type
    if Camera_Parameters_Str.Type == 'PERSP':
        bpy.data.cameras[name].lens = Camera_Parameters_Str.Value
    elif Camera_Parameters_Str.Type == 'ORTHO':
        bpy.data.cameras[name].ortho_scale = Camera_Parameters_Str.Value

def Convert_Ax_Str2Id(ax: str) -> int:
    """
    Description:
        Convert a string axis letter to an identification number.
        
    Args:
        (1) ax [string]: Axis name.
        
    Returns:
        (1) parameter [int]: Identification number.
    """
    
    return {
        'X': 0,
        'Y': 1,
        'Z': 2,
        'ALL': -1
    }[ax]

def Insert_Key_Frame(name: str, property: str, frame: int, index: str):
    """
    Description:
        Insert a keyframe with the given property.
        
    Args:
        (1) name [string]: Name of the main object.
        (2) property [string]: The path to the property that is supposed to be the key (matrix_basis, location, rotation_euler, scale, etc.)
        (3) frame [int]: The frame in which the keyframe is inserted.
        (4) index [string]: The index of the property to be the key.
                            Note:
                                The "location" property has 3 parts: [x, y, z][Vector<float>]
                                1\ index = 'ALL' -> all parts
                                2\ index = 'X' -> x part, etc.
    """
    
    # Convert a string letter to an identification number.
    index_id_num = Convert_Ax_Str2Id(index)
    
    if property == 'matrix_basis':
        bpy.data.objects[name].keyframe_insert('location', frame=frame, index=index_id_num)
        bpy.data.objects[name].keyframe_insert('rotation_euler', frame=frame, index=index_id_num)
        bpy.data.objects[name].keyframe_insert('scale', frame=frame, index=index_id_num)
    else:
        bpy.data.objects[name].keyframe_insert(property, frame=frame, index=index_id_num)  

def Transform_Object_To_Wireframe(name: str, thickness: float) -> None:
    """
    Description:
        The wireframe modifier transforms a mesh object into a wireframe model with a defined
        size (thickness).

    Args:
        (1) name [string]: Name of the main object.
        (2) thickness [float]: The size (thickness) of the wireframes.
    """

    # Deselect all objects in the current scene.
    Deselect_All()

    # Select the desired object in the scene.
    bpy.data.objects[name].select_set(True)
    # Add the modifier (wireframe) and set the desired thickness.
    bpy.ops.object.modifier_add(type='WIREFRAME')
    bpy.context.object.modifiers['Wireframe'].thickness = thickness
    bpy.ops.object.modifier_apply(modifier="Wireframe")
    # Release the object from the selection.
    bpy.data.objects[name].select_set(False)

def Attach_Viewpoints(viewpoint_name: str, Robot_Parameters_Str: Lib.Parameters.Robot.Robot_Parameters_Str) -> None:
    """
    Decription:
        Attach viewpoints with the correct transformation to an object joints.
        
    Args:
        (1) viewpoint_name [string]: Name of the main object.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.   
    """

    # Set the rotation mode.
    bpy.data.objects[viewpoint_name].rotation_mode = 'ZYX'

    for _, (name, T_i_zero_cfg) in enumerate(zip(Robot_Parameters_Str.Theta.Name, Robot_Parameters_Str.T.Zero_Cfg)):
        # Duplication of objects and/or object hierarchy (if exists).
        Duplicate_Object(viewpoint_name, bpy.data.objects[name].name)

        # Set the name of the new waipoint.
        #   (name + '_' + identification)
        viewpoint_name_new = viewpoint_name + '_' + bpy.data.objects[name].name

        # Set the target (desired) viewpoint position in the current joint.
        bpy.data.objects[viewpoint_name_new].matrix_basis = T_i_zero_cfg.Transpose().all()
        # Connecting a child object (viewpoint_i) to a parent object (joint_i).
        bpy.data.objects[viewpoint_name_new].parent = bpy.data.objects[name]
        # Reset (null) the object matrix.
        bpy.data.objects[viewpoint_name_new].matrix_basis = Transformation.Get_Matrix_Identity(4) 

def Add_Viewpoints(viewpoint_name: str, T: tp.List[tp.List[tp.List[float]]]) -> None:
    """
    Decription:
        Add viewpoints with the correct transformation.
        
    Args:
        (1) viewpoint_name [string]: Name of the main object.
        (2) T [Matrix<float> nx(4x4)]: Configuration homogeneous matrix of each joint.
                                       Note:
                                        Where n is the number of joints.
    """

    # Set the rotation mode.
    bpy.data.objects[viewpoint_name].rotation_mode = 'ZYX'

    for i, T_i in enumerate(T):
        # Duplication of objects and/or object hierarchy (if exists).
        Duplicate_Object(viewpoint_name, f'Joint_{i}')

        # Set the name of the new waipoint.
        #   (name + '_' + identification)
        viewpoint_name_new = viewpoint_name + '_' + f'Joint_{i}'
        
        # Set the target (desired) viewpoint position in the current joint.
        bpy.data.objects[viewpoint_name_new].location       = T_i.p.all()
        bpy.data.objects[viewpoint_name_new].rotation_euler = T_i.Get_Rotation('ZYX').all()

def Is_Point_Inside_Object(name: str, point: tp.List[float], max_distance: float) -> bool:
    """
    Description:
        Determine if the point is located inside the mesh object.

    Args:
        (1) name [string]: Name of the mesh object.
        (2) point [Vector<float>]: Input point (x, y, z).
        (3) max_distance [float]: Maximum distance.

    Returns:
        (1) parameter [bool]: The result is 'True' if the point is inside the object, and 
                              'False' if it is not.

    Example:
        Is_Point_Inside_Mesh(np.array([0.0, 0.0, 0.0]), 1.0e+5, 'Cube')
    """

    # Find the nearest point on the object.
    _, nearest_point, normal, _ = bpy.data.objects[name].closest_point_on_mesh(point, distance=max_distance)

    return not((np.array(nearest_point) - point) @ normal < 0.0)

def Get_Vertices_From_Object(name: str) -> tp.List[float]:
    """
    Description:
        Get (x, y, z) positions of the vertices of the mesh object.

    Args:
        (1) name [string]: Name of the mesh object.

    Returns:
        (1) parameter [Vector<float>]: Vector (list) of given vertices.
    """

    return [bpy.data.objects[name].matrix_world @ vertex_i.co for vertex_i in bpy.data.objects[name].data.vertices]

def Set_Object_Origin(name: str, T: tp.List[tp.List[float]]) -> None:
    """
    Description:
        Set the origin (position / rotation) of the individual objects.

    Args:
        (1) name [string]: Name of the mesh object. 
        (2) T [Matrix<float> 4x4]: Origin of the object (homogeneous transformation matrix).
    """

    # Select an object.
    bpy.data.objects[name].select_set(True)

    # Set the position of the cursor and the origin of the 
    # object.
    bpy.context.scene.cursor.matrix = T
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    # Deselect an object and update the layer.
    bpy.data.objects[name].select_set(False)
    bpy.context.view_layer.update()

def Generate_Convex_Polyhedron_From_Data(name: str, data: tp.List[float], material_properties: tp.Tuple[tp.List[float], float]) -> None:
    """
    Description:
        Generate a simplified convex polyhedron from the input data (x, y, z).

    Args:
        (1) name [string]: Name of the mesh object.
        (2) data [Vector<float>: Input coordinates (x, y, z).
        (3) material_properties [Dictionary {'RGBA': Vector<float>, 'alpha': float}]: Properties of the object material.
    """
    
    # Create a new material and set the material color of the object.
    material = bpy.data.materials.new(f'{name}_mat')
    material.use_nodes = True
    material.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = material_properties['RGBA']

    # Create a mesh from the input data.
    mesh = bpy.data.meshes.new(f'{name}_Convex_Polyhedron')
    mesh.from_pydata(data, [], [])
    mesh.update()

    # Create an object from the mesh and add it to the scene 
    # collection.
    object = bpy.data.objects.new(name, mesh)
    bpy.data.collections['Collection'].objects.link(object)

    # Add material to the object.
    object.data.materials.append(material)

    """
    Description:
       Generate a simplified convex polyhedron in the scene.
    """
    bpy.context.view_layer.objects.active = bpy.data.objects[name]
    bpy.ops.object.mode_set(mode='EDIT')
    # Transform the points into a convex polyhedron.
    bpy.ops.mesh.convex_hull()
    # Simplification of the convex polyhedron.
    bpy.ops.mesh.dissolve_limited()
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.objects.active = None

    # Set the transparency of the object material.
    if material_properties['alpha'] < 1.0:
        Set_Object_Material_Transparency(name, material_properties['alpha'])

def __Get_BMesh_Data_From_Object(name: str) -> bmesh.types.BMesh:
    """
    Description:
        Get the BMesh representation (data) from the mesh object.

    Args:
        (1) name [string]: Name of the mesh object. 

    Returns:
        (1) parameter [bmesh.types.BMesh]: BMesh representation (data) of the object.
    """

    bmesh_data = bmesh.new()
    bmesh_data.from_mesh(bpy.context.scene.objects[name].data)
    bmesh_data.transform(bpy.context.scene.objects[name].matrix_world)

    return bmesh_data

def Check_Overlap_Objects(name_1: str, name_2: str) -> bool:
    """
    Description:
        A function to check if two mesh objects overlap or not. 

    Args:
        (1 - 2) name_{i} [string]: Name of the mesh objects.

    Returns:
        (1) parameter [bool]: Objects overlap (True) or not (False).
    """

    # Get a BVH tree based on BMesh data.
    bhv_obj_1 = mathutils.bvhtree.BVHTree.FromBMesh(__Get_BMesh_Data_From_Object(name_1))
    bhv_obj_2 = mathutils.bvhtree.BVHTree.FromBMesh(__Get_BMesh_Data_From_Object(name_2))

    # Test the overlap of the two meshes.
    if bhv_obj_1.overlap(bhv_obj_2):
        return True
    else: 
        return False
