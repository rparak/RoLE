# BPY (Blender as a python) [pip3 install bpy]
import bpy
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Blender/Utilities
import Lib.Blender.Utilities
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

class Poly_3D_Cls(object):
    """
    Description:
        Visualization of a 3-D (dimensional) polyline.
        
    Initialization of the Class:
        Args:
            (1) name [string]: The name of the polyline (object).
            (2) curve_properties [Dictionary {'bevel_depth': float, 
                                              'color': Vector<float>}]: Properties of the curve (polyline).
                                                                        Note:
                                                                            bevel_depth: Radius of the bevel geometry.
                                                                            color: The color of the curve.
            (3) point_properties [Dictionary {'visibility': bool, 'radius': float, 'color': Vector<float>}]: Properties of curve points.
            
            Explanations:
                (2: bevel_depth)
                    Radius of the bevel geometry.
                (2: color)
                    The color of the curve.
                (3: visibility)
                    Visibility of points on the curve.
                (3: radius)
                    Radius of points.
                (3: color)
                    The color of the points.
    
        Example:
            Initialization:
                Cls = Poly_3D_Cls(name, {'bevel_depth': bevel_depth, 'color': color}, 
                                 {'visibility': visibility, 'radius': radius, 'color': color})
            
            Features:
                # Remove the object (polyline) from the scene.
                Cls.Remove()
                
                # Initialize the size (length) of the polyline data set.
                Cls.Initialization(size_of_ds)
                
                # Add coordinates (x, y, z points) to the polyline.
                Cls.Add(index, coordinates)
                
                # Visualization of a 3-D (dimensional) polyline in the scene.
                Cls.Visualization()
                
                # Animation of a 3-D (dimensional) polyline in the scene.
                Cls.Animation(frame_start, frame_end)
    """

    def __init__(self, name: str, curve_properties: tp.Tuple[float, tp.List[float]], point_properties: tp.Tuple[tp.Union[None, bool], float, tp.List[float]]) -> None: 
        # << PRIVATE >> #
        self.__name = name
        self.__visibility_points = point_properties['visibility']
        # Remove the object (hierarchy) from the scene, if it exists
        Lib.Blender.Utilities.Remove_Object(self.__name)
        # Create a polyline data-block.
        self.__data_block = bpy.data.curves.new(self.__name, type='CURVE')
        self.__data_block.dimensions = '3D'
        self.__data_block.bevel_depth = curve_properties['bevel_depth']
        # Radius of points.
        self.__radius_points = point_properties['radius']
        # The collection of splines in this polyline data-block.
        self.__polyline = self.__data_block.splines.new('POLY')
        # Creation of new material.
        #   Curve.
        self.__material_curve = bpy.data.materials.new(self.__data_block.name + 'mat')
        self.__material_curve.use_nodes = True
        self.__material_curve.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = curve_properties['color']
        #   Points (Sphere).
        if self.__visibility_points == True:
            self.__material_sphere = bpy.data.materials.new(self.__data_block.name + 'mat')
            self.__material_sphere.use_nodes = True
            self.__material_sphere.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = point_properties['color']
    
    def Initialization(self, size_of_ds: int) -> None:
        """
        Description:
            Initialize the size (length) of the polyline data set.
            
        Args:
            (1) size_of_ds [int]: The size (length) of the data set.
        """
        
        self.__polyline.points.add(size_of_ds - 1)
        
    def Add(self, index: int, coordinates: tp.List[float]) -> None:
        """
        Description:
            Add coordinates (x, y, z points) to the polyline.
            
        Args:
            (1) index [int]: Index of the current added point.
            (2) coordinates [Vector<float> 1x3]: Coordinates (x, y, z) of the polyline.
        """

        x, y, z = coordinates

        self.__polyline.points[index].co = (x, y, z, 1)

        if self.__visibility_points == True:
            self.__Add_Sphere(index, x, y, z)

    def __Add_Sphere(self, index: int, x: float, y: float, z: float) -> None:
        """
        Description:
            Add coordinates (x,y,z) of a spherical object (point).

        Args:
            (1) index [int]: Index of the current added point.
            (2 - 4) x, y, z [float]: Coordinates of the point (spherical object).
        """
        
        # Creation a spherical object with a defined position.
        Lib.Blender.Utilities.Create_Primitive('Sphere', {'Radius': self.__radius_points, 'Location': (x, y, z)})
        bpy.ops.object.shade_smooth()
        # Change the name and material of the object.
        bpy.context.active_object.name = f'Point_{index}'
        bpy.context.active_object.active_material = self.__material_sphere
    
    def Visualization(self):
        """
        Description:
            Visualization of a 3-D (dimensional) polyline in the scene.
        """
        
        bpy.data.objects.new(self.__data_block.name, self.__data_block).data.materials.append(self.__material_curve)
        bpy.data.collections['Collection'].objects.link(bpy.data.objects.new(self.__data_block.name, self.__data_block))

        if self.__visibility_points == True:
            # Deselect all objects in the current scene.
            Lib.Blender.Utilities.Deselect_All()
            # Selection of the main object (curve)
            bpy.ops.object.select_pattern(pattern=f'{self.__name}.*', extend=False)

            for i in range(len(self.__polyline.points)):
                # Connecting a child object (point_i .. n) to a parent object (curve).
                bpy.data.objects[f'Point_{i}'].parent = bpy.data.objects[f'{bpy.context.selected_objects[0].name}']

    def Animation(self, frame_start: float, frame_end: float) -> None:
        """
        Description:
            Animation of a 3-D (dimensional) polyline in the scene.
            
        Args:
            (1) frame_start, frame_end [float]: The frame (start, end) in which the keyframe is inserted.
        """
        
        # Specifies how the tart / end bevel factor maps to the spline.
        bpy.data.objects[self.__data_block.name].data.bevel_factor_mapping_start = 'SPLINE'
        bpy.data.objects[self.__data_block.name].data.bevel_factor_mapping_end   = 'SPLINE'
        
        # Factor that determines the location of the bevel:
        #   0.0 (0 %), 1.0 (100 %)
        bpy.data.objects[self.__data_block.name].data.bevel_factor_end = 0.0
        bpy.data.objects[self.__data_block.name].data.keyframe_insert(data_path='bevel_factor_end', frame=frame_start, index=-1)
        bpy.data.objects[self.__data_block.name].data.bevel_factor_end = 1.0
        bpy.data.objects[self.__data_block.name].data.keyframe_insert(data_path='bevel_factor_end', frame=frame_end, index=-1)

class Robot_Cls(object):
    """
    Description:
        ...

    Initialization of the Class:
        Args:
            ...

        Example:
            Initialization:
                # Assignment of the variables.
                ...

                # Initialization of the class.
                ...

            Features:
                # Properties of the class.
                ...

                # Functions of the class.
                ...
    """

    def __init__(self, Robot_Parameters_Str: Lib.Parameters.Robot.Robot_Parameters_Str):
        # << PRIVATE >> #
        self.__Robot_Parameters_Str = Robot_Parameters_Str
        self.__Robot_Parameters_Str.T.Base = HTM_Cls(bpy.data.objects[self.__Robot_Parameters_Str.Name].matrix_basis, 
                                                     np.float32)
        self.__Robot_Parameters_Str.T.Zero_Cfg = Kinematics.Get_Individual_Joint_Configuration(self.__Robot_Parameters_Str.Theta.Zero, 'Modified', 
                                                                                               self.__Robot_Parameters_Str)[1]
        
        self.__Viewpoint_EE_Name = f'{self.__Robot_Parameters_Str.Name}_Viewpoint_EE'

        # add visibility viewpoint here and function will be private

    @property
    def Theta_0(self):
        return self.__Robot_Parameters_Str.Theta.Zero
    
    @property
    def Theta(self):
        return Lib.Blender.Utilities.Get_Absolute_Joint_Position(self.__Robot_Parameters_Str)

    @property
    def T_EE(self):
        return Kinematics.Forward_Kinematics(self.Theta, 'Modified', self.__Robot_Parameters_Str)[1]

    def Viewpoint_Visibility(self, state: bool) -> None:
        """
        Description:
            Function to enable and disable the visibility of the end-effector viewpoint.
        
        Args:
            (1) state [bool]: Enable (True) / Disable (False).  
        """

        if Lib.Blender.Utilities.Object_Exist(self.__Viewpoint_EE_Name):
            # ...
            Lib.Blender.Utilities.Set_Object_Transformation(self.__Viewpoint_EE_Name, 
                                                            self.T_EE)
            # ...
            Lib.Blender.Utilities.Object_Visibility(self.__Viewpoint_EE_Name, state)

    def Get_Random_Theta(self):
        """
        Description:

        """

        Theta_Random = np.array([0.0] * self.__Robot_Parameters_Str.Theta.Zero.size, np.float32)
        for i in range(Theta_Random.size):
            Theta_Random[i] = np.float32(np.random.uniform(self.__Robot_Parameters_Str.Theta.Limit[i][0], 
                                                           self.__Robot_Parameters_Str.Theta.Limit[i][1]))
            
        return Theta_Random

    def Set_Absolute_Joint_Position(self, theta):
        Lib.Blender.Utilities.Set_Absolute_Joint_Position(theta, self.__Robot_Parameters_Str)
        Lib.Blender.Utilities.Set_Object_Transformation(self.__Viewpoint_EE_Name, self.T_EE)
