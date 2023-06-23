# BPY (Blender as a python) [pip3 install bpy]
import bpy
# Typing (Support for type hints)
import typing as tp

def Deselect_All() -> None:
    """
    Description:
        Deselect all objects in the current scene.
    """
    
    for obj in bpy.context.selected_objects:
        bpy.data.objects[obj.name].select_set(False)
    bpy.context.view_layer.update()

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
    bpy.context.view_layer.objects.active = bpy.data.objects[name]
    bpy.ops.object.mode_set(mode='EDIT')
    # Add the modifier (wireframe) and set the desired thickness.
    bpy.ops.object.modifier_add(type='WIREFRAME')
    bpy.context.object.modifiers['Wireframe'].thickness = thickness
    # Release the object from the selection.
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.objects.active = None
    bpy.data.objects[name].select_set(False)

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
    mesh = bpy.data.meshes.new(f'{name}')
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