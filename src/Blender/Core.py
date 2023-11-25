# BPY (Blender as a python) [pip3 install bpy]
import bpy
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   Blender
#       ../Blender/Utilities
import Blender.Utilities
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot
#       ../RoLE/Parameters/Mechanism
import RoLE.Parameters.Mechanism
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core as Kinematics
#       ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation
#       ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#       ../RoLE/Trajectory/Utilities
import RoLE.Trajectory.Utilities
#       ../RoLE/Primitives/Core
from RoLE.Primitives.Core import Box_Cls
#       ../RoLE/Primitives/Core
from RoLE.Collider.Core import OBB_Cls

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
                                                                                'bevel_depth': Radius of the bevel geometry.
                                                                                'color': The color of the curve.
            (3) point_properties [Dictionary {'visibility': bool, 'radius': float, 
                                              'color': Vector<float>}]: Properties of curve points.
                                                                            Note:
                                                                                'visibility': Visibility of points on the curve.
                                                                                'radius': Radius of points.
                                                                                'color': The color of the points.
                                                                                
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
        Blender.Utilities.Remove_Object(self.__name)
        # Create a polyline data-block.
        self.__data_block = bpy.data.curves.new(self.__name, type='CURVE')
        self.__data_block.dimensions = '3D'
        self.__data_block.bevel_depth = curve_properties['bevel_depth']
        # Radius and color of points.
        self.__radius_points = point_properties['radius']
        self.__color_points = point_properties['color']
        # The collection of splines in this polyline data-block.
        self.__polyline = self.__data_block.splines.new('POLY')
        # Creation of new material.
        #   Curve.
        self.__material_curve = bpy.data.materials.new(self.__data_block.name + 'mat')
        self.__material_curve.use_nodes = True
        self.__material_curve.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = curve_properties['color']
    
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
        
        # Properties of the created object.
        point_properties = {'transformation': {'Radius': self.__radius_points, 'Location': [x, y, z]}, 
                            'material': {'RGBA': self.__color_points, 'alpha': 1.0}}
        
        # Creation a spherical object with a defined position.
        Blender.Utilities.Create_Primitive('Sphere', f'Point_{index}', point_properties)
        bpy.ops.object.shade_smooth()
    
    def Visualization(self):
        """
        Description:
            Visualization of a 3-D (dimensional) polyline in the scene.
        """
        
        bpy.data.objects.new(self.__data_block.name, self.__data_block).data.materials.append(self.__material_curve)
        bpy.data.collections['Collection'].objects.link(bpy.data.objects.new(self.__data_block.name, self.__data_block))

        if self.__visibility_points == True:
            # Deselect all objects in the current scene.
            Blender.Utilities.Deselect_All()
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

class Mechanism_Cls(object):
    """
    Description:
        A class for working with a mechanism object in a Blender scene.

    Initialization of the Class:
        Args:
            (1) Mechanism_Parameters_Str [Mechanism_Parameters_Str(object)]: The structure of the main parameters of the mechanism.
            (2) properties [Dictionary {'fps': int, 
                                        'visibility': {'Viewpoint_EE': bool, 'Colliders': bool, 
                                                       'Ghost': bool}}]: The properties of the mechanism structure 
                                                                         in the Blender environment.
                                                                            Note:
                                                                                'fps': The FPS (Frames Per Seconds) value.
                                                                                'visibility': The state to enable/disable the visibility 
                                                                                              of additional mechanism objects.

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the SMC LEFB25UNZS 1400C mechanism.
                Mechanism_Parameters_Str = RoLE.Parameters.Mechanism.SMC_LEFB25_1400_0_1_Str
                #   The properties of the mechanism structure in the Blender environment.
                properties = {'fps': 100, 'visibility': {'Viewpoint_EE': False, 'Colliders': False, 
                                                         'Ghost': False}}

                # Initialization of the class.
                Cls = Mechanism_Cls(Mechanism_Parameters_Str, visibility)

            Features:
                # Properties of the class.
                Cls.Parameters
                Cls.Theta_0; Cls.T_EE

                # Functions of the class.
                Cls.Set_Absolute_Joint_Position(0.0, 0.0, 1.0)
    """
        
    def __init__(self, Mechanism_Parameters_Str: RoLE.Parameters.Mechanism.Mechanism_Parameters_Str, properties: tp.Dict) -> None:
        try:
            assert Blender.Utilities.Object_Exist(f'{Mechanism_Parameters_Str.Name}_ID_{Mechanism_Parameters_Str.Id:03}') == True
            
            # << PRIVATE >> #
            self.__Mechanism_Parameters_Str = Mechanism_Parameters_Str
            self.__external_object_id = 0
            self.__properties = properties
            self.__fps = self.__properties['fps']

            # Modified the name of the mechanism structure. Addition of mechanism structure Id (identification number).
            self.__name = f'{Mechanism_Parameters_Str.Name}_ID_{Mechanism_Parameters_Str.Id:03}'

            # Set the transformation of the mechanism structure in Blender to the transformation obtained from the parameters.
            Blender.Utilities.Set_Object_Transformation(self.__name, self.__Mechanism_Parameters_Str.T.Base)

            # Rotation axis sequence configuration (e.g. 'ZYX', 'QUATERNION', etc.)
            self.__axes_sequence_cfg = 'ZYX'

            # Set the FPS (frames per second) value of the simulation in Blender.
            bpy.context.scene.render.fps = self.__fps
            
            # Initialization of the class to generate trajectory.
            self.__Trapezoidal_Cls = RoLE.Trajectory.Utilities.Trapezoidal_Profile_Cls(delta_time=1.0/self.__fps)

            # Enable or disable the visibility of additional objects of the mechanism.
            #   1\ End-effector viewpoint.
            if Blender.Utilities.Object_Exist(f'Viewpoint_EE_{self.__name}'):
                Blender.Utilities.Object_Visibility(f'Viewpoint_EE_{self.__name}', self.__properties['visibility']['Viewpoint_EE'])
            #   2\ Colliders (base + joints).
            for _, collider_name in enumerate(np.concatenate((list(self.__Mechanism_Parameters_Str.Collider.Base), 
                                                              list(self.__Mechanism_Parameters_Str.Collider.Theta)), dtype=str)):
                if Blender.Utilities.Object_Exist(collider_name):
                    Blender.Utilities.Object_Visibility(collider_name, self.__properties['visibility']['Colliders'])

                # Set to the default colour.
                Blender.Utilities.Set_Object_Material_Color(collider_name, [0.70, 0.85, 0.60, 1.0])
                Blender.Utilities.Set_Object_Material_Transparency(collider_name, 0.2)
            #   3\  The 'ghost' of the movable part of the mechanism.
            self.__mechanism_th_ghost_name = None
            for _, th_name in enumerate(self.__Mechanism_Parameters_Str.Collider.Theta.keys()):
                th_ghost_name = f"{th_name.removesuffix(f'Collider_{self.__name}')}Ghost_{self.__name}"
                if Blender.Utilities.Object_Exist(th_ghost_name):
                    Blender.Utilities.Object_Visibility(th_ghost_name, self.__properties['visibility']['Ghost'])

                if 'Joint' in th_ghost_name:
                    self.__mechanism_th_ghost_name = th_ghost_name

                # Set to the default colour.
                Blender.Utilities.Set_Object_Material_Color(th_ghost_name, [0.1, 0.1, 0.1, 1.0])
                Blender.Utilities.Set_Object_Material_Transparency(th_ghost_name, 0.2)
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] The mechanism object named <{Mechanism_Parameters_Str.Name}_ID_{Mechanism_Parameters_Str.Id:03}> does not exist in the current scene.')

    def Change_Color(self, color: tp.List[float]):
        # in progress ...
        for _, collider_name in enumerate(np.concatenate((list(self.__Mechanism_Parameters_Str.Collider.Base), 
                                                          list(self.__Mechanism_Parameters_Str.Collider.Theta),
                                                          list(self.__Mechanism_Parameters_Str.Collider.External)), dtype=str)):
            Blender.Utilities.Set_Object_Material_Color(collider_name, np.append(color[0:3], [1.0]))
            Blender.Utilities.Set_Object_Material_Transparency(collider_name, color[-1])

    @property
    def Parameters(self) -> RoLE.Parameters.Mechanism.Mechanism_Parameters_Str:
        """
        Description:
            Get the structure of the main parameters of the mechanism.

        Returns:
            (1) parameter [Mechanism_Parameters_Str(object)]: The structure of the main parameters of the mechanism.
        """
        
        return self.__Mechanism_Parameters_Str
    
    @property
    def Theta_0(self) -> float:
        """
        Description:
            Get the zero (home) absolute position of the joint in radians/meter.

        Returns:
            (1) parameter [Vector<float>]: Zero (home) absolute joint position in radians / meters.
        """

        return self.__Mechanism_Parameters_Str.Theta.Zero

    @property
    def Theta(self) -> float:
        """
        Description:
            Get the absolute position of the mechanism joint.

        Returns:
            (1) parameter [Vector<float>]: Current absolute joint position in radians / meters.
        """

        # Convert a string axis letter to an identification number.
        ax_i_id_num = Blender.Utilities.Convert_Ax_Str2Id(self.__Mechanism_Parameters_Str.Theta.Axis)

        if self.__Mechanism_Parameters_Str.Theta.Type == 'R':
            # Identification of joint type: R - Revolute
            theta = bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].rotation_euler[ax_i_id_num]
        elif self.__Mechanism_Parameters_Str.Theta.Type == 'P':
            # Identification of joint type: P - Prismatic
            theta = bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].location[ax_i_id_num]

        return theta * self.__Mechanism_Parameters_Str.Theta.Direction
        
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the homogeneous transformation matrix of the mechanism slider.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix of the mechanism slider.
        """

        # Get the actual homogeneous transformation matrix of the mechanism slider.
        T_Slider = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].matrix_basis, 
                                                                         np.float64)

        return self.__Mechanism_Parameters_Str.T.Base @ T_Slider @ self.__Mechanism_Parameters_Str.T.Shuttle
    
    def __Update(self) -> None:
        """
        Description:
            Update the scene.
        """

        bpy.context.view_layer.update()

    def Add_External_Object(self, T: tp.List[tp.List[float]], color: tp.List[float], 
                            size: tp.List[float], enable_collision: bool) -> None:
        """
        Description:
            A function to add external objects (cube) to the Blender environment.

        Args:
            (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the object.
            (2) color [Vector<float> 1x4]: The color of the object.
                                            Note:
                                                Format: rgba(red, green, blue, alpha)
            (3) size [Vector<float> 1x3]: The size of the object.
            (4) enable_collision [bool]: Information on whether or not the object is to be exposed 
                                         to collisions
        """

        if isinstance(T, (list, np.ndarray)):
            T = Transformation.Homogeneous_Transformation_Matrix_Cls(T, np.float64)

        # Set the properties of the added object.
        box_properties = {'transformation': {'Size': 1.0, 
                                             'Scale': size, 
                                             'Location': [0.0, 0.0, 0.0]}, 
                          'material': {'RGBA': color, 'alpha': color[-1]}}
        #  Object name.
        object_name = f'Object_ID_{self.__external_object_id}'
        
        # Create a primitive three-dimensional object with additional properties.
        Blender.Utilities.Create_Primitive('Cube', object_name, box_properties)

        # Set the object transformation.
        Blender.Utilities.Set_Object_Transformation(object_name, T)

        # Enable collision of the added object.
        if enable_collision == True:
            # Add a collider (type OBB) as a part of the mechanism structure.
            self.__Mechanism_Parameters_Str.Collider.External[object_name] = OBB_Cls(Box_Cls([0.0, 0.0, 0.0],
                                                                                             size))
            # Oriented Bounding Box (OBB) transformation according to the input homogeneous 
            # transformation matrix.
            self.__Mechanism_Parameters_Str.Collider.External[object_name].Transformation(T)

        self.__external_object_id += 1
        
    def Remove_All_External_Objects(self) -> None:
        """
        Description:
            A function to remove all external objects from the Blender environment that were added 
            using the 'Add_External_Object' function of the class.

            Note:
                The function also removes external colliders added to the mechanism structure.
        """

        i = 0
        while True:
            # Obtain the name of the object.
            object_name = f'Object_ID_{i}'

            if Blender.Utilities.Object_Exist(object_name) == True:
                Blender.Utilities.Remove_Object(object_name)
            else:
                break     
            i += 1

        self.__external_object_id = 0; self.__Mechanism_Parameters_Str.Collider.External = {}

    def __Set_Collider_Color(self, info: tp.List[float]):
        """
        Description:
            Function to set the color of the colliders, which depends on the information vector.

            Note:
                The index of the vector is equal to the index of the collider. If the index of the information 
                vector is 'True', that means the collider object is in collision.

        Args:
            (1) info [Vector<bool> 1xk]: The vector of errors where collisions have occurred between joints of the mechanism structure 
                                         or with external objects.
                                            Note:
                                                Where k is the number of all colliders of the mechanism structure.
        """

        # Get the name of the colliders.
        collider_name = np.append(list(self.__Mechanism_Parameters_Str.Collider.Base), 
                                  list(self.__Mechanism_Parameters_Str.Collider.Theta), dtype=str)

        for _, (info_i, collider_name_i) in enumerate(zip(info, collider_name)):
            if info_i == True:
                color = [0.85, 0.60, 0.60, 0.2]
            else:
                color = [0.70, 0.85, 0.60, 0.2]

            Blender.Utilities.Set_Object_Material_Color(collider_name_i, np.append(color[0:3], [1.0]))
            Blender.Utilities.Set_Object_Material_Transparency(collider_name_i, color[-1])

    def __Set_Ghost_Structure_Color(self, color: tp.List[float]) -> None:
        """
        Description:
            Function to set the color of the auxiliary mechanism structure, which is represented as a "ghost".

        Args:
            (1) color [None or Vector<float> 1x4]: The color of the object.
                                                    Note:
                                                        Format: rgba(red, green, blue, alpha).
        """

        for _, th_name in enumerate(self.__Mechanism_Parameters_Str.Collider.Theta.keys()):
            th_ghost_name = f"{th_name.removesuffix(f'Collider_{self.__name}')}Ghost_{self.__name}"
            Blender.Utilities.Set_Object_Material_Color(th_ghost_name, np.append(color[0:3], [1.0]))
            Blender.Utilities.Set_Object_Material_Transparency(th_ghost_name, color[-1])

    def __Reset_Ghost_Structure(self, theta: tp.List[float]) -> None:
        """
        Description:
            Function to reset the absolute position of the auxiliary mechanism structure, which is represented as a "ghost".

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters. Used only in individual 
                                           mode.
                                            Note:
                                                Where n is the number of joints.
        """

        bpy.data.objects[self.__mechanism_th_ghost_name].rotation_mode = self.__axes_sequence_cfg
        if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:

            # Change of axis direction in individual joints.
            th = theta * self.__Mechanism_Parameters_Str.Theta.Direction

            if self.__Mechanism_Parameters_Str.Theta.Type == 'R':
                # Identification of joint type: R - Revolute
                bpy.data.objects[self.__mechanism_th_ghost_name].rotation_euler = (self.__Mechanism_Parameters_Str.T.Slider @ 
                                                                                   Transformation.Get_Rotation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                                                                                                      th)).Get_Rotation(self.__axes_sequence_cfg).all()
            elif self.__Mechanism_Parameters_Str.Theta.Type == 'P':
                # Identification of joint type: P - Prismatic
                bpy.data.objects[self.__mechanism_th_ghost_name].location = (self.__Mechanism_Parameters_Str.T.Slider @ 
                                                                             Transformation.Get_Translation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                                                                                                   th)).p.all()

        # Update the scene.
        self.__Update()

    def Reset(self, mode: str, theta: tp.Union[None, float] = None) -> bool:
        """
        Description:
            Function to reset the absolute position of the mechanism joint from the selected mode.

            Note:
                The Zero/Home modes are predefined in the mechanism structure and the Individual mode is used 
                to set the individual position defined in the function input parameter.

        Args:
            (1) mode [string]: Possible modes to reset the absolute position of the joint.
            (2) theta [float]: Desired absolute joint position in radians / meters.

        Returns:
            (1) parameter [bool]: The result is 'True' if the mechanism is in the desired position,
                                  and 'False' if it is not.
        """

        try:
            assert mode in ['Zero', 'Home', 'Individual']

            if mode == 'Individual':
                assert isinstance(theta, float) == True
                
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Mechanism_Parameters_Str.Theta.Home

            bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].rotation_mode = self.__axes_sequence_cfg
            if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta_internal <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:

                # Change of axis direction in individual joints.
                th = theta_internal * self.__Mechanism_Parameters_Str.Theta.Direction

                if self.__Mechanism_Parameters_Str.Theta.Type == 'R':
                    # Identification of joint type: R - Revolute
                    bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].rotation_euler = (self.__Mechanism_Parameters_Str.T.Slider @ 
                                                                                                Transformation.Get_Rotation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                                                                                                                   th)).Get_Rotation(self.__axes_sequence_cfg).all()
                elif self.__Mechanism_Parameters_Str.Theta.Type == 'P':
                    # Identification of joint type: P - Prismatic
                    bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].location = (self.__Mechanism_Parameters_Str.T.Slider @ 
                                                                                            Transformation.Get_Translation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                                                                                                                  th)).p.all()
            else:
                # Update the scene.
                self.__Update()
                print(f'[WARNING] The desired input joint {th} is out of limit.')
                return False
    
            # Update the scene.
            self.__Update()
            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            if mode not in ['Zero', 'Home', 'Individual']:
                print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')
            if isinstance(theta, float) == False:
                print('[ERROR] Incorrect value type in the input variable theta. The input variable must be of type float.')

    def Set_Absolute_Joint_Position(self, theta: float, t_0: float, t_1: float) -> bool:
        """
        Description:
            Set the absolute position of the mechanism joint.

        Args:
            (1) theta [float]: Desired absolute joint position in radians / meters.
            (2) t_0 [float]: Animation start time in seconds.
            (3) t_1 [float]: Animation stop time in seconds.

         Returns:
            (1) parameter [bool]: The result is 'True' if the mechanism is in the desired position,
                                  and 'False' if it is not.
        """
        
        try:
            assert isinstance(theta, float)

            # Generation of position trajectories from input parameters.
            (theta_arr, _, _) = self.__Trapezoidal_Cls.Generate(self.Theta, theta, 0.0, 0.0, 
                                                                t_0, t_1)

            for _, (t_i, theta_arr_i) in enumerate(zip(self.__Trapezoidal_Cls.t, theta_arr)):
                bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].rotation_mode = self.__axes_sequence_cfg
                if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta_arr_i <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:

                    # Change of axis direction in individual joints.
                    th = theta_arr_i * self.__Mechanism_Parameters_Str.Theta.Direction

                    if self.__Mechanism_Parameters_Str.Theta.Type == 'R':
                        # Identification of joint type: R - Revolute
                        bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].rotation_euler = (self.__Mechanism_Parameters_Str.T.Slider @ 
                                                                                                    Transformation.Get_Rotation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                                                                                                                       th)).Get_Rotation(self.__axes_sequence_cfg).all()
                    elif self.__Mechanism_Parameters_Str.Theta.Type == 'P':
                        # Identification of joint type: P - Prismatic
                        bpy.data.objects[self.__Mechanism_Parameters_Str.Theta.Name].location = (self.__Mechanism_Parameters_Str.T.Slider @ 
                                                                                                Transformation.Get_Translation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                                                                                                                      th)).p.all()
                        
                        # Insert a keyframe of the object (Joint_{i}) into the frame at time t(i). 
                        Blender.Utilities.Insert_Key_Frame(self.__Mechanism_Parameters_Str.Theta.Name, 'matrix_basis', np.int32(t_i * self.__fps), 'ALL')
                else:
                    # Update the scene.
                    self.__Update()
                    print(f'[WARNING] The desired input joint {theta_arr_i} is out of limit.')
                    return False

            # Update the scene.
            self.__Update()
            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect value type in the input variable theta. The input variable must be of type float.')

class Robot_Cls(object):
    """
    Description:
        A class for working with a robotic arm object in a Blender scene.

    Initialization of the Class:
        Args:
            (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
            (2) properties [Dictionary {'fps': int, 
                                        'visibility': {'Viewpoint_EE': bool, 'Colliders': bool, 
                                                       'Workspace': bool, 'Ghost': bool}}]: The properties of the robot structure 
                                                                                            in the Blender environment.
                                                                                                Note:
                                                                                                    'fps': The FPS (Frames Per Seconds) value.
                                                                                                    'visibility': The state to enable/disable the visibility 
                                                                                                                  of additional robot objects.
                                        
        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the ABB IRB 120 robot.
                Robot_Parameters_Str = RoLE.Parameters.Robot.ABB_IRB_120_Str
                #   The properties of the robot structure in the Blender environment.
                properties = {'fps': 100, 'visibility': {'Viewpoint_EE': False, 'Colliders': False, 
                                                         'Workspace': False, 'Ghost': False}}

                # Initialization of the class.
                Cls = Robot_Cls(Robot_Parameters_Str, properties)

            Features:
                # Properties of the class.
                Cls.Parameters
                Cls.Theta_0; Cls.T_EE

                # Functions of the class.
                Cls.Set_Absolute_Joint_Position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 1.0)
    """

    def __init__(self, Robot_Parameters_Str: RoLE.Parameters.Robot.Robot_Parameters_Str, properties: tp.Dict) -> None:
        try:
            assert Blender.Utilities.Object_Exist(f'{Robot_Parameters_Str.Name}_ID_{Robot_Parameters_Str.Id:03}') == True
            
            # << PRIVATE >> #
            self.__Robot_Parameters_Str = Robot_Parameters_Str
            self.__external_object_id = 0
            self.__properties = properties
            self.__fps = self.__properties['fps']

            # Modified the name of the robot structure. Addition of robot structure Id (identification number).
            self.__name = f'{Robot_Parameters_Str.Name}_ID_{Robot_Parameters_Str.Id:03}'

            # Set the transformation of the robot structure in Blender to the transformation obtained from the parameters.
            Blender.Utilities.Set_Object_Transformation(self.__name, self.__Robot_Parameters_Str.T.Base)

            # Get the zero configuration of the homogeneous matrix of each joint using forward kinematics. 
            self.__Robot_Parameters_Str.T.Zero_Cfg = Kinematics.Get_Individual_Joint_Configuration(self.__Robot_Parameters_Str.Theta.Zero, 'Modified', 
                                                                                                   self.__Robot_Parameters_Str)[1]
            
            # Rotation axis sequence configuration (e.g. 'ZYX', 'QUATERNION', etc.)
            self.__axes_sequence_cfg = 'ZYX'

            # Set the FPS (frames per second) value of the simulation in Blender.
            bpy.context.scene.render.fps = self.__fps
            
            # Initialization of the class to generate trajectory.
            self.__Polynomial_Cls = RoLE.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=1.0/self.__fps)

            # Enable or disable the visibility of additional objects of the mechanism.
            #   1\ End-effector viewpoint.
            if Blender.Utilities.Object_Exist(f'Viewpoint_EE_{self.__name}'):
                Blender.Utilities.Object_Visibility(f'Viewpoint_EE_{self.__name}', self.__properties['visibility']['Viewpoint_EE'])
            #   2\ Colliders (base + joints).
            for _, collider_name in enumerate(np.concatenate((list(self.__Robot_Parameters_Str.Collider.Base), 
                                                              list(self.__Robot_Parameters_Str.Collider.Theta)), dtype=str)):
                if Blender.Utilities.Object_Exist(collider_name):
                    Blender.Utilities.Object_Visibility(collider_name, self.__properties['visibility']['Colliders'])

                # Set to the default colour.
                Blender.Utilities.Set_Object_Material_Color(collider_name, [0.70, 0.85, 0.60, 1.0])
                Blender.Utilities.Set_Object_Material_Transparency(collider_name, 0.2)
            #   3\ Workspace.
            if Blender.Utilities.Object_Exist(f'Workspace_{self.__name}'):
                Blender.Utilities.Object_Visibility(f'Workspace_{self.__name}', self.__properties['visibility']['Workspace'])
            #   4\  The 'ghost' of the movable part of the robot.
            self.__robot_th_ghost_name = []
            for _, th_name in enumerate(self.__Robot_Parameters_Str.Collider.Theta.keys()):
                th_ghost_name = f"{th_name.removesuffix(f'Collider_{self.__name}')}Ghost_{self.__name}"
                if Blender.Utilities.Object_Exist(th_ghost_name):
                    Blender.Utilities.Object_Visibility(th_ghost_name, self.__properties['visibility']['Ghost'])

                if 'Joint' in th_ghost_name:
                    self.__robot_th_ghost_name.append(th_ghost_name)

                # Set to the default colour.
                Blender.Utilities.Set_Object_Material_Color(th_ghost_name, [0.1, 0.1, 0.1, 1.0])
                Blender.Utilities.Set_Object_Material_Transparency(th_ghost_name, 0.2)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] The robot object named <{Robot_Parameters_Str.Name}_ID_{Robot_Parameters_Str.Id:03}> does not exist in the current scene.')

    @property
    def Parameters(self) -> RoLE.Parameters.Robot.Robot_Parameters_Str:
        """
        Description:
            Get the structure of the main parameters of the robot.

        Returns:
            (1) parameter [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        """
        
        return self.__Robot_Parameters_Str
    
    @property
    def Theta_0(self) -> tp.List[float]:
        """
        Description:
            Get the zero (home) absolute position of the joint in radians/meter.

        Returns:
            (1) parameter [Vector<float>]: Zero (home) absolute joint position in radians / meters.
        """

        return self.__Robot_Parameters_Str.Theta.Zero
    
    def __Get_Zero_Joint_Cfg(self) -> tp.List[tp.List[tp.List[float]]]:
        """
        Description:
            Get the zero configuration of each joint using a modified forward kinematics calculation method.

        Returns:
            (1) parameter [Matrix<float> nx(4x4)]: Zero configuration of each joint.
                                                    Note:
                                                        Where n is the number of joints.
        """

        T_i = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64); T_zero_cfg = []
        for i, (th_i, dh_i, th_i_type, th_ax_i) in enumerate(zip(self.__Robot_Parameters_Str.Theta.Zero, self.__Robot_Parameters_Str.DH.Modified, 
                                                                 self.__Robot_Parameters_Str.Theta.Type, self.__Robot_Parameters_Str.Theta.Axis)):
            # Forward kinematics using modified DH parameters.
            if th_i_type == 'R':
                # Identification of joint type: R - Revolute
                T_i = T_i @ Kinematics.DH_Modified(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
            elif th_i_type == 'P':
                # Identification of joint type: P - Prismatic
                if th_ax_i == 'Z':
                    T_i = T_i @ Kinematics.DH_Modified(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
                else:
                    # Translation along the X axis.
                    T_i = T_i @ Kinematics.DH_Modified(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

            # Addition of a homogeneous matrix configuration in the current 
            # episode (joint absolute position i).
            if self.__Robot_Parameters_Str.Theta.Zero.size - 1 == i:
                T_zero_cfg.append(T_i @ self.__Robot_Parameters_Str.T.End_Effector)
            else:
                T_zero_cfg.append(T_i)

        return T_zero_cfg
    
    @property
    def Theta(self) -> tp.List[float]:
        """
        Description:
            Get the absolute positions of the robot's joints.

        Returns:
            (1) parameter [Vector<float> 1xn]: Current absolute joint position in radians / meters.
                                                Note:
                                                    Where n is the number of joints.
        """

        # Get the zero configuration of each joint.
        T_zero_cfg = self.__Get_Zero_Joint_Cfg()

        th = np.zeros(self.__Robot_Parameters_Str.Theta.Zero.shape)
        for i, (th_i_name, T_i_zero_cfg, ax_i, th_i_type) in enumerate(zip(self.__Robot_Parameters_Str.Theta.Name, T_zero_cfg, 
                                                                           self.__Robot_Parameters_Str.Theta.Axis, self.__Robot_Parameters_Str.Theta.Type)):   
            # Convert a string axis letter to an identification number.
            ax_i_id_num = Blender.Utilities.Convert_Ax_Str2Id(ax_i)

            if th_i_type == 'R':
                # Identification of joint type: R - Revolute
                #   The actual orientation of the joint in iteration i.
                th_actual = bpy.data.objects[th_i_name].rotation_euler[ax_i_id_num]
                #   The initial orientation of the joint in iteration i.
                th_init   = T_i_zero_cfg.Get_Rotation(self.__axes_sequence_cfg).all()[ax_i_id_num]

                if (th_init - th_actual) > Mathematics.CONST_MATH_PI:
                    th[i] = Mathematics.CONST_MATH_PI * 2 - (th_init - th_actual)
                else:    
                    th[i] = th_actual - th_init

            elif th_i_type == 'P':
                # Identification of joint type: P - Prismatic
                #   The actual translation of the joint in iteration i.
                th_actual = bpy.data.objects[th_i_name].location[ax_i_id_num]
                #   The initial translation of the joint in iteration i.
                th_init   = T_i_zero_cfg.p.all()[ax_i_id_num]
    
                th[i] = th_actual - th_init

        return th * self.__Robot_Parameters_Str.Theta.Direction
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the homogeneous transformation matrix of the robot end-effector.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix of the End-Effector.
        """

        return Kinematics.Forward_Kinematics(self.Theta, 'Fast', self.__Robot_Parameters_Str)[1]

    def __Update(self) -> None:
        """
        Description:
            Update the scene.
        """

        bpy.context.view_layer.update()
        
    def Add_External_Object(self, T: tp.List[tp.List[float]], color: tp.List[float], 
                            size: tp.List[float], enable_collision: bool) -> None:
        """
        Description:
            A function to add external objects (cube) to the Blender environment.

        Args:
            (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the object.
            (2) color [Vector<float> 1x4]: The color of the object.
                                            Note:
                                                Format: rgba(red, green, blue, alpha)
            (3) size [Vector<float> 1x3]: The size of the object.
            (4) enable_collision [bool]: Information on whether or not the object is to be exposed 
                                         to collisions
        """

        if isinstance(T, (list, np.ndarray)):
            T = Transformation.Homogeneous_Transformation_Matrix_Cls(T, np.float64)

        # Set the properties of the added object.
        box_properties = {'transformation': {'Size': 1.0, 
                                             'Scale': size, 
                                             'Location': [0.0, 0.0, 0.0]}, 
                          'material': {'RGBA': color, 'alpha': color[-1]}}
        #  Object name.
        object_name = f'Object_ID_{self.__external_object_id}'
        
        # Create a primitive three-dimensional object with additional properties.
        Blender.Utilities.Create_Primitive('Cube', object_name, box_properties)

        # Set the object transformation.
        Blender.Utilities.Set_Object_Transformation(object_name, T)

        # Enable collision of the added object.
        if enable_collision == True:
            # Add a collider (type OBB) as a part of the robotic arm structure.
            self.__Robot_Parameters_Str.Collider.External[object_name] = OBB_Cls(Box_Cls([0.0, 0.0, 0.0],
                                                                                         size))
            # Oriented Bounding Box (OBB) transformation according to the input homogeneous 
            # transformation matrix.
            self.__Robot_Parameters_Str.Collider.External[object_name].Transformation(T)

        self.__external_object_id += 1

    def Remove_All_External_Objects(self) -> None:
        """
        Description:
            A function to remove all external objects from the Blender environment that were added 
            using the 'Add_External_Object' function of the class.

            Note:
                The function also removes external colliders added to the robotic structure.
        """

        i = 0
        while True:
            # Obtain the name of the object.
            object_name = f'Object_ID_{i}'

            if Blender.Utilities.Object_Exist(object_name) == True:
                Blender.Utilities.Remove_Object(object_name)
            else:
                break     
            i += 1

        self.__external_object_id = 0; self.__Robot_Parameters_Str.Collider.External = {}

    def __Set_Collider_Color(self, info: tp.List[float]):
        """
        Description:
            Function to set the color of the colliders, which depends on the information vector.

            Note:
                The index of the vector is equal to the index of the collider. If the index of the information 
                vector is 'True', that means the collider object is in collision.

        Args:
            (1) info [Vector<bool> 1xk]: The vector of errors where collisions have occurred between joints of the robotic structure 
                                         or with external objects.
                                            Note:
                                                Where k is the number of all colliders of the robotic structure.
        """

        # Get the name of the colliders.
        collider_name = np.append(list(self.__Robot_Parameters_Str.Collider.Base), 
                                  list(self.__Robot_Parameters_Str.Collider.Theta), dtype=str)

        for _, (info_i, collider_name_i) in enumerate(zip(info, collider_name)):
            if info_i == True:
                color = [0.85, 0.60, 0.60, 0.2]
            else:
                color = [0.70, 0.85, 0.60, 0.2]

            Blender.Utilities.Set_Object_Material_Color(collider_name_i, np.append(color[0:3], [1.0]))
            Blender.Utilities.Set_Object_Material_Transparency(collider_name_i, color[-1])

    def __Set_Ghost_Structure_Color(self, color: tp.List[float]) -> None:
        """
        Description:
            Function to set the color of the auxiliary robot structure, which is represented as a "ghost".

        Args:
            (1) color [None or Vector<float> 1x4]: The color of the object.
                                                    Note:
                                                        Format: rgba(red, green, blue, alpha).
        """

        for _, th_name in enumerate(self.__Robot_Parameters_Str.Collider.Theta.keys()):
            th_ghost_name = f"{th_name.removesuffix(f'Collider_{self.__name}')}Ghost_{self.__name}"
            Blender.Utilities.Set_Object_Material_Color(th_ghost_name, np.append(color[0:3], [1.0]))
            Blender.Utilities.Set_Object_Material_Transparency(th_ghost_name, color[-1])

    def __Reset_Ghost_Structure(self, theta: tp.List[float]) -> None:
        """
        Description:
            Function to reset the absolute position of the auxiliary robot structure, which is represented as a "ghost".

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters. Used only in individual 
                                           mode.
                                            Note:
                                                Where n is the number of joints.
        """

        # Get the zero configuration of each joint.
        T_zero_cfg = self.__Get_Zero_Joint_Cfg()

        for _, (th_i, th_i_name, T_i_zero_cfg, ax_i, th_i_type, th_i_dir) in enumerate(zip(theta, self.__robot_th_ghost_name, T_zero_cfg, 
                                                                                           self.__Robot_Parameters_Str.Theta.Axis, self.__Robot_Parameters_Str.Theta.Type, 
                                                                                           self.__Robot_Parameters_Str.Theta.Direction)): 
            bpy.data.objects[th_i_name].rotation_mode = self.__axes_sequence_cfg

            # Change of axis direction in individual joints.
            th_new = th_i * th_i_dir

            if th_i_type == 'R':
                # Identification of joint type: R - Revolute
                bpy.data.objects[th_i_name].rotation_euler = (T_i_zero_cfg @ Transformation.Get_Rotation_Matrix(ax_i, th_new)).Get_Rotation(self.__axes_sequence_cfg).all()
            elif th_i_type == 'P':
                # Identification of joint type: P - Prismatic
                bpy.data.objects[th_i_name].location = (Transformation.Get_Translation_Matrix(ax_i, th_new) @ T_i_zero_cfg).p.all()

        # Update the scene.
        self.__Update()

    def Reset(self, mode: str, theta: tp.Union[None, tp.List[float]] = None) -> bool:
        """
        Description:
            Function to reset the absolute position of the robot joints from the selected mode.

            Note:
                The Zero/Home modes are predefined in the robot structure and the Individual mode is used 
                to set the individual position defined in the function input parameter.

        Args:
            (1) mode [string]: Possible modes to reset the absolute position of the joints.
            (2) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters. Used only in individual 
                                           mode.
                                            Note:
                                                Where n is the number of joints.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """

        try:
            assert mode in ['Zero', 'Home', 'Individual']

            if mode == 'Individual':
                assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size
                
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Robot_Parameters_Str.Theta.Home

            # Get the zero configuration of each joint.
            T_zero_cfg = self.__Get_Zero_Joint_Cfg()

            for i, (th_i, th_i_name, T_i_zero_cfg, th_i_limit, ax_i, th_i_type, th_i_dir) in enumerate(zip(theta_internal, self.__Robot_Parameters_Str.Theta.Name, T_zero_cfg, 
                                                                                                           self.__Robot_Parameters_Str.Theta.Limit, self.__Robot_Parameters_Str.Theta.Axis, 
                                                                                                           self.__Robot_Parameters_Str.Theta.Type, self.__Robot_Parameters_Str.Theta.Direction)): 
                bpy.data.objects[th_i_name].rotation_mode = self.__axes_sequence_cfg
                if th_i_limit[0] <= th_i <= th_i_limit[1]:
                    # Change of axis direction in individual joints.
                    th_new = th_i * th_i_dir

                    if th_i_type == 'R':
                        # Identification of joint type: R - Revolute
                        bpy.data.objects[th_i_name].rotation_euler = (T_i_zero_cfg @ Transformation.Get_Rotation_Matrix(ax_i, th_new)).Get_Rotation(self.__axes_sequence_cfg).all()
                    elif th_i_type == 'P':
                        # Identification of joint type: P - Prismatic
                        bpy.data.objects[th_i_name].location = (Transformation.Get_Translation_Matrix(ax_i, th_new) @ T_i_zero_cfg).p.all()
                else:
                    # Update the scene.
                    self.__Update()
                    print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                    return False

            # Update the scene.
            self.__Update()
            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            if mode not in ['Zero', 'Home', 'Individual']:
                print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')
            if self.__Robot_Parameters_Str.Theta.Zero.size != theta.size:
                print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], t_0: float, t_1: float) -> bool:
        """
        Description:
            Set the absolute position of the robot joints.

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
            (2) t_0 [float]: Animation start time in seconds.
            (3) t_1 [float]: Animation stop time in seconds.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """

        try:
            assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size

            # Generation of multi-axis position trajectories from input parameters.
            theta_arr = []
            for _, (th_actual, th_desired) in enumerate(zip(self.Theta, theta)):
                (theta_arr_i, _, _) = self.__Polynomial_Cls.Generate(th_actual, th_desired, 0.0, 0.0, 0.0, 0.0,
                                                                     t_0, t_1)
                theta_arr.append(theta_arr_i)

            for _, (t_i, theta_arr_i) in enumerate(zip(self.__Polynomial_Cls.t, np.array(theta_arr, dtype=np.float64).T)):
                # Get the zero configuration of each joint.
                T_zero_cfg = self.__Get_Zero_Joint_Cfg()

                for i, (th_i, th_i_name, T_i_zero_cfg, th_i_limit, ax_i, th_i_type, th_i_dir) in enumerate(zip(theta_arr_i, self.__Robot_Parameters_Str.Theta.Name, T_zero_cfg, 
                                                                                                               self.__Robot_Parameters_Str.Theta.Limit, self.__Robot_Parameters_Str.Theta.Axis, 
                                                                                                               self.__Robot_Parameters_Str.Theta.Type, self.__Robot_Parameters_Str.Theta.Direction)): 
                    bpy.data.objects[th_i_name].rotation_mode = self.__axes_sequence_cfg
                    if th_i_limit[0] <= th_i <= th_i_limit[1]:
                        # Change of axis direction in individual joints.
                        th_new = th_i * th_i_dir

                        if th_i_type == 'R':
                            # Identification of joint type: R - Revolute
                            bpy.data.objects[th_i_name].rotation_euler = (T_i_zero_cfg @ Transformation.Get_Rotation_Matrix(ax_i, th_new)).Get_Rotation(self.__axes_sequence_cfg).all()
                        elif th_i_type == 'P':
                            # Identification of joint type: P - Prismatic
                            bpy.data.objects[th_i_name].location = (Transformation.Get_Translation_Matrix(ax_i, th_new) @ T_i_zero_cfg).p.all()

                        # Insert a keyframe of the object (Joint_{i}) into the frame at time t(i). 
                        Blender.Utilities.Insert_Key_Frame(th_i_name, 'matrix_basis', np.int32(t_i * self.__fps), 'ALL')

                    else:
                        # Update the scene.
                        self.__Update()
                        print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                        return False

            # Update the scene.
            self.__Update()
            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')

    def Get_Inverse_Kinematics_Solution(self, T: tp.List[tp.List[float]]) -> tp.Dict[bool, tp.List[float]]:
        """
        Description:
            ...
        """

        if isinstance(T, (list, np.ndarray)):
            T = Transformation.Homogeneous_Transformation_Matrix_Cls(T, np.float64)

        # ...
        # self.__Reset_Ghost_Structure()

        # Set the color of the 'ghost' structure.
        if True:
            self.__Set_Ghost_Structure_Color([0.1, 0.1, 0.1, 0.2])
        else:
            self.__Set_Ghost_Structure_Color([0.85, 0.60, 0.60, 0.2])
