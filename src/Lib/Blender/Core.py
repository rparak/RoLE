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
import Lib.Transformation.Core as Transformation
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

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

class Mechanism_Cls(object):
    def __init__(self, viewpoint_visibility: bool) -> None:
        pass

class Robot_Cls(object):
    """
    Description:
        A class for working with a robotic arm object in a Blender scene.

    Initialization of the Class:
        Args:
            (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
            (2) viewpoint_visibility [bool]: The state (enable/disable) of the robot end-effector viewpoint object visibility.

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the ABB IRB 120 robot.
                Robot_Parameters_Str = Lib.Parameters.Robot.ABB_IRB_120_Str
                viewpoint_visibility = True

                # Initialization of the class.
                Cls = Robot_Cls(Robot_Parameters_Str, viewpoint_visibility)

            Features:
                # Properties of the class.
                Cls.Parameters
                Cls.Theta_0, Cls.T_EE

                # Functions of the class.
                Cls.Set_Absolute_Joint_Position([0.0. 0.0, 0.0, 0.0, 0.0, 0.0])
    """

    def __init__(self, Robot_Parameters_Str: Lib.Parameters.Robot.Robot_Parameters_Str, viewpoint_visibility: bool) -> None:
        try:
            assert Lib.Blender.Utilities.Object_Exist(f'{Robot_Parameters_Str.Name}_ID_{Robot_Parameters_Str.Id:03}') == True
            
            # << PRIVATE >> #
            self.__Robot_Parameters_Str = Robot_Parameters_Str
            # Modified the name of the robot structure. Addition of robot structure Id (identification number).
            self.__name = f'{Robot_Parameters_Str.Name}_ID_{Robot_Parameters_Str.Id:03}'
            # Get the homogeneous transformation matrix of the robot based on the position of the robot structure in Blender.
            self.__Robot_Parameters_Str.T.Base = Transformation.Homogeneous_Transformation_Matrix_Cls(bpy.data.objects[self.__name].matrix_basis, 
                                                                                                      np.float32)
            # Get the zero configuration of the homogeneous matrix of each joint using forward kinematics. 
            self.__Robot_Parameters_Str.T.Zero_Cfg = Kinematics.Get_Individual_Joint_Configuration(self.__Robot_Parameters_Str.Theta.Zero, 'Modified', 
                                                                                                   self.__Robot_Parameters_Str)[1]
            
            # Rotation axis sequence configuration (e.g. 'ZYX', 'QUATERNION', etc.)
            self.__axes_sequence_cfg = 'ZYX'
            
            # Enable or disable the visibility of the end-effector viewpoint.
            self.__viewpoint_visibility = viewpoint_visibility
            self.__Viewpoint_EE_Name = f'{self.__name}_Viewpoint_EE'
            if Lib.Blender.Utilities.Object_Exist(self.__Viewpoint_EE_Name):
                # Enable / disable the visibility of an object.
                Lib.Blender.Utilities.Object_Visibility(self.__Viewpoint_EE_Name, self.__viewpoint_visibility)
                # Set the transformation of the viewpoint object.
                #   The object transformation will be set to the end-effector of the robot structure.
                Lib.Blender.Utilities.Set_Object_Transformation(self.__Viewpoint_EE_Name, self.T_EE)
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] The robot object named <{Robot_Parameters_Str.Name}_ID_{Robot_Parameters_Str.Id:03}> does not exist in the current scene.')

    @property
    def Parameters(self) -> Lib.Parameters.Robot.Robot_Parameters_Str:
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

        T_i = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float32); T_zero_cfg = []
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
            (1) parameter [Vector<float>]: Current absolute joint position in radians / meters.
        """

        # Get the zero configuration of each joint.
        T_zero_cfg = self.__Get_Zero_Joint_Cfg()

        th = np.zeros(self.__Robot_Parameters_Str.Theta.Zero.shape)
        for i, (th_i_name, T_i_zero_cfg, ax_i, th_i_type) in enumerate(zip(self.__Robot_Parameters_Str.Theta.Name, T_zero_cfg, 
                                                                           self.__Robot_Parameters_Str.Theta.Axis, self.__Robot_Parameters_Str.Theta.Type)):   
            # Convert a string axis letter to an identification number.
            ax_i_id_num = Lib.Blender.Utilities.Convert_Ax_Str2Id(ax_i)

            if th_i_type == 'R':
                # Identification of joint type: R - Revolute
                #   The actual orientation of the joint in iteration i.
                th_actual = bpy.data.objects[th_i_name].rotation_euler[ax_i_id_num]
                #   The initial orientation of the joint in iteration i.
                th_init   = T_i_zero_cfg.Get_Rotation(self.__axes_sequence_cfg).all()[ax_i_id_num]

                if (th_actual - th_init) > Mathematics.CONST_MATH_PI:
                    th[i] = (th_actual - th_init) - Mathematics.CONST_MATH_PI * 2
                else:    
                    th[i] = th_actual - th_init
            elif th_i_type == 'P':
                # Identification of joint type: P - Prismatic
                #   The actual translation of the joint in iteration i.
                th_actual = bpy.data.objects[th_i_name].location[ax_i_id_num]
                #   The initial translation of the joint in iteration i.
                th_init   = T_i_zero_cfg.p.all()[ax_i_id_num]
    
                th[i] = th_actual - th_init

        return th

    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the homogeneous transformation matrix of the robot end-effector.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
        """
        return Kinematics.Forward_Kinematics(self.Theta, 'Modified', self.__Robot_Parameters_Str)[1]

    def __Update(self) -> None:
        """
        Description:
            Update the scene.
        """

        bpy.context.view_layer.update()

    def Reset(self, mode: str) -> None:
        """
        Description:
            Function to reset the absolute position of the robot joints from the selected mode.

        Args:
            (1) mode [string]: Possible modes to reset the absolute position of the joints.
        """

        try:
            assert mode in ['Zero', 'Home']

            if mode == 'Zero':
                self.Set_Absolute_Joint_Position(self.Theta_0)
            else:
                self.Set_Absolute_Joint_Position(self.__Robot_Parameters_Str.Theta.Home)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[INFO] Incorrect reset mode selected. The selected mode must be chosen from the two options (Zero, Home).')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float]) -> bool:
        """
        Description:
            Set the absolute position of the robot joints.

        Args:
            (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.

        Returns:
            (1) parameter [bool]: The result is 'True' if the required absolute joint positions 
                                  are within the limits, and 'False' if they are not.
        """
        try:
            assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size

            # Get the zero configuration of each joint.
            T_zero_cfg = self.__Get_Zero_Joint_Cfg()

            for i, (th_i, th_i_name, T_i_zero_cfg, th_i_limit, ax_i, th_i_type) in enumerate(zip(theta, self.__Robot_Parameters_Str.Theta.Name, 
                                                                                                 T_zero_cfg, self.__Robot_Parameters_Str.Theta.Limit,
                                                                                                 self.__Robot_Parameters_Str.Theta.Axis, self.__Robot_Parameters_Str.Theta.Type)): 
                bpy.data.objects[th_i_name].rotation_mode = self.__axes_sequence_cfg
                if th_i_limit[0] <= th_i <= th_i_limit[1]:
                    if th_i_type == 'R':
                        # Identification of joint type: R - Revolute
                        bpy.data.objects[th_i_name].rotation_euler = (T_i_zero_cfg @ Transformation.Get_Rotation_Matrix(ax_i, th_i)).Get_Rotation(self.__axes_sequence_cfg).all()
                    elif th_i_type == 'P':
                        # Identification of joint type: P - Prismatic
                        bpy.data.objects[th_i_name].location = (T_i_zero_cfg @ Transformation.Get_Translation_Matrix(ax_i, th_i)).p.all()
                else:
                    # Update the scene.
                    self.__Update()
                    # Reset the absolute position of the robot joints to the 'Zero'.
                    self.Reset('Zero')
                    print(f'[INFO] The desired input joint {th_i} in index {i} is out of limit.')
                    return False

            # If the viewpoint visibility is enabled, set the transformation of the object 
            # to the end-effector of the robot.
            if self.__viewpoint_visibility == True:
                Lib.Blender.Utilities.Set_Object_Transformation(self.__Viewpoint_EE_Name, self.T_EE)

            # Update the scene.
            self.__Update()

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')