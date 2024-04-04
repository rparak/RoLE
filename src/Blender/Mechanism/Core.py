"""
## =========================================================================== ## 
MIT License
Copyright (c) 2023 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
## =========================================================================== ## 
Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak
File Name: ../Blender/Mechanism/Core.py
## =========================================================================== ## 
"""

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
#       ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation
#       ../RoLE/Trajectory/Utilities
import RoLE.Trajectory.Utilities
#       ../RoLE/Primitives/Core
from RoLE.Primitives.Core import Box_Cls
#       ../RoLE/Primitives/Core
from RoLE.Collider.Core import OBB_Cls

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

        return np.round(theta * self.__Mechanism_Parameters_Str.Theta.Direction, 5)
        
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
        collider_name = np.concatenate((list(self.__Mechanism_Parameters_Str.Collider.Base), 
                                        list(self.__Mechanism_Parameters_Str.Collider.Theta)), dtype=str)

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
            Function to set the color of the auxiliary mechanism structure, which is represented as a 'ghost'.

        Args:
            (1) color [None or Vector<float> 1x4]: The color of the object.
                                                    Note:
                                                        Format: rgba(red, green, blue, alpha).
        """

        for _, th_name in enumerate(self.__Mechanism_Parameters_Str.Collider.Theta.keys()):
            th_ghost_name = f"{th_name.removesuffix(f'Collider_{self.__name}')}Ghost_{self.__name}"
            Blender.Utilities.Set_Object_Material_Color(th_ghost_name, np.append(color[0:3], [1.0]))
            Blender.Utilities.Set_Object_Material_Transparency(th_ghost_name, color[-1])

    def __Reset_Ghost_Structure(self, theta: tp.List[float]) -> bool:
        """
        Description:
            Function to reset the absolute position of the auxiliary mechanism structure, which is represented as a 'ghost'.

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        Returns:
            (1) parameter [bool]: The result is 'True' if the mechanism is in the desired position,
                                  and 'False' if it is not.
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
        else:
            return False

        # Update the scene.
        self.__Update()
        return True

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

    def __Is_External_Collision(self, theta: float) -> tp.List[bool]:
        """
        Description:
            A function to obtain information about whether a part of the mechanism structure collides 
            with external objects.

        Args:
            (1) theta [float]: Desired absolute joint position in radians / meters.

        Returns:
            (1) parameter [bool]: Information about whether there are collisions between joints.
            (2) parameter [Vector<bool> 1xk]: A vector of information where a collision occurred between the joints of the mechanism structure.
                                                Note:
                                                    Where k is the number of all colliders of the mechanism structure.
        """

        # Get a list of colliders.
        Base_Collider = list(self.__Mechanism_Parameters_Str.Collider.Base.values()); Theta_Collider = list(self.__Mechanism_Parameters_Str.Collider.Theta.values())
        External_Collider = list(self.__Mechanism_Parameters_Str.Collider.External.values())

        # Transformation of the base collider according to the input homogeneous transformation matrix.
        Base_Collider[0].Transformation(self.__Mechanism_Parameters_Str.T.Base)

        if self.__Mechanism_Parameters_Str.Theta.Type == 'R':
            T = self.__Mechanism_Parameters_Str.T.Slider @ Transformation.Get_Rotation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, theta)
        elif self.__Mechanism_Parameters_Str.Theta.Type == 'P':
            T = self.__Mechanism_Parameters_Str.T.Slider @ Transformation.Get_Translation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, theta)

        # Transformation of the colliders according to the input homogeneous transformation matrix.
        for _, th_collider_i in enumerate(Theta_Collider):
            th_collider_i.Transformation(T)

        # Concatenate all colliders (base, joint) into single array according to a predefined constraint.
        All_Colliders = np.concatenate((Base_Collider, Theta_Collider))

        # Check whether the external 3D primitives (bounding boxes AABB, OBB) overlap or do not overlap 
        # with the mechanism structure.
        collision_info = np.zeros(All_Colliders.size, dtype=bool)
        for i, collider_i in enumerate(All_Colliders):
            for _, external_collider_i in enumerate(External_Collider):
                if collider_i.Overlap(external_collider_i) == True:
                    # Set the part of the mechanism structure where the collision occurs.
                    collision_info[i] = True

        return (collision_info.any() == True, collision_info)
    
    def Get_Inverse_Kinematics_Solution(self, T: tp.List[tp.List[float]], enable_ghost: bool) -> tp.Tuple[bool, float]:
        """
        Description:
            A function to compute the solution of the inverse kinematics (IK) of the mechanism structure. 

        Args:
            (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the desired TCP position.
            (2) enable_ghost [bool]: Enable visibility of the auxiliary mechanism structure, which 
                                     is represented as a 'ghost'.
                                        Note:
                                            To make the auxiliary mechanism structure visible, the 'ghost' parameter must 
                                            also be enabled in the class properties.

        Returns:
            (1) parameter [bool]: The result is 'True' if the inverse kinematics (IK) has a solution, and 'False' if 
                                  it does not.
            (2) parameter [float]: Obtained solution of the absolute positions of the joints in radians / meters.
        """

        if isinstance(T, (list, np.ndarray)):
            T = Transformation.Homogeneous_Transformation_Matrix_Cls(T, np.float64)

        # Convert a string axis letter to an identification number.
        ax_i_id_num = Blender.Utilities.Convert_Ax_Str2Id(self.__Mechanism_Parameters_Str.Theta.Axis)

        # Get the absolute position of the mechanism joint.
        if self.__Mechanism_Parameters_Str.Theta.Type == 'R':
            theta = T.Get_Rotation('ZYX').all()[ax_i_id_num]
        elif self.__Mechanism_Parameters_Str.Theta.Type == 'P':
            theta = T.p.all()[ax_i_id_num]

        # Check whether a part of the mechanism structure collides with external objects.
        (is_collision, collision_info) = self.__Is_External_Collision(theta)

        # Set the color of the colliders.
        #   Note:
        #       'Red': Collision.
        #       'Green': No collision.
        self.__Set_Collider_Color(collision_info)

        # Check whether the inverse kinematics (IK) has a solution or not.
        #   Conditions:
        #       1\ Within limits.
        #       2\ Collision-free.
        if self.__Reset_Ghost_Structure(theta) == True and is_collision == False:
            successful = True
        else:
            successful = False

        # Set the color of the 'ghost' structure.
        if enable_ghost == True and self.__properties['visibility']['Ghost'] == True:
            if successful:
                self.__Set_Ghost_Structure_Color([0.1, 0.1, 0.1, 0.2])
            else:
                self.__Set_Ghost_Structure_Color([0.85, 0.60, 0.60, 0.2])

        return (successful, theta)