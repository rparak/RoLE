# Typing (Support for type hints)
import typing as tp
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Script:
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
# Custom Script:
#   ../Lib/URDF/Utilities
import Lib.URDF.Utilities as Utilities
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Utilities/File_IO
import Lib.Utilities.File_IO as File_IO
#   ../Lib/Utilities/MOI
import Lib.Utilities.MOI as MOI

class URDF_Generator_Cls(object):
    """
    Description:
      ...

    Initialization of the Class:
        Args:
          (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
          (2) use_mesh [bool]: Use a mesh to represent a visual/collision object. Otherwise, use a sphere to 
                              represent the object.
          (3) is_external_axis [bool]: Is the external axis part of the robot or not. For example, a linear track.
          (4) rgba [Vector<float> 1x4]: 

        Example:
            Initialization:
                # Assignment of the variables.
                

                # Initialization of the class.
                Cls = 

            Features:
                # Functions of the class.
                Cls.
    
    """
    # URDF (Unified Robotics Description Format).
    def __init__(self, Robot_Str: Parameters.Robot_Parameters_Str, use_mesh: bool, is_external_axis: bool, rgba: str) -> None:
        # ...
        self.__Robot_Str = Robot_Str
        self.__use_mesh = use_mesh
        self.__is_external_axis = is_external_axis
        self.__rgba = ' '.join(str(x) for x in rgba)

        # Get the defined physical properties of the robotic structure.
        self.__Robot_Properties = Utilities.Get_Physical_Properties(self.__Robot_Str.Name)

        # ...
        if self.__is_external_axis == True:
          self.__configuration = {'Base_0': '', 'External': '', 'Base_n': '', 'Core': [], 'EE': ''}
        else:
          self.__configuration = {'Base_0': '', 'Core': [], 'EE': ''}
    

    def __Object_Geometry(self, name: str) -> tp.Tuple[str, str]:
      if self.__use_mesh == True:
          return {'visual': f'<mesh filename="/Mesh/Visual/{name}.stl"/>', 
                  'collision': f'<mesh filename="/Mesh/Visual/{name}.stl"/>'}
      else:
          return {'visual': '<sphere radius="0.01"/>', 
                  'collision': '<sphere radius="0.01"/>'}
      
    def __Configure_Base_0(self) -> str:
      # ...
      obj_geometry = self.__Object_Geometry('Base')

      # ...
      moi = MOI.Cube_MOI(self.__Robot_Properties['mass'][0], self.__Robot_Str.Collider[0].Size)

      # ...
      bbox_origin = ((-1) *  self.__Robot_Str.Collider[0].Origin) + [0.0, 0.0, 0.0]

      return f'''  <!-- Configuration of the part called 'Base 0'. -->
  <link name="base_link">
    <visual>
      <geometry>
        {obj_geometry['visual']}
      </geometry>
      <material name="Custom_Color">
        <color rgba="{self.__rgba}"/>
      </material>
    </visual>
    <collision>
      <geometry>
        {obj_geometry['collision']}
      </geometry>
    </collision>
    <inertial>
      <mass value="{self.__Robot_Properties['mass'][0]}"/>
      <origin rpy="0.0 0.0 0.0" xyz="{bbox_origin[0]:.05f} {bbox_origin[1]:.05f} {bbox_origin[2]:.05f}"/>
      <inertia ixx="{moi['I_xx']:.10f}" ixy="0.0" ixz="0.0" iyy="{moi['I_yy']:.10f}" iyz="0.0" izz="{moi['I_zz']:.10f}"/>
    </inertial>
  </link>'''

    def __Configure_Base_n(self, n, i, child_link: str, parent_link: str) -> str:
      # ...
      obj_geometry = self.__Object_Geometry(f'Base_{n}')

      # ...
      moi = MOI.Cube_MOI(self.__Robot_Properties['mass'][i], self.__Robot_Str.Collider[i].Size)

      # ...
      bbox_origin = ((-1) *  self.__Robot_Str.Collider[i].Origin) + [0.0, 0.0, 0.0]

      return f'''  <!-- Configuration of the part called 'Base {n}'. -->
  <joint name="base_{n}" type="fixed">
    <parent link="{parent_link}"/>
    <child link="{child_link}"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
  </joint>
  <link name="{child_link}">
    <visual>
      <geometry>
        {obj_geometry['visual']}
      </geometry>
      <material name="Custom_Color">
        <color rgba="{self.__rgba}"/>
      </material>
    </visual>
    <collision>
      <geometry>
        {obj_geometry['collision']}
      </geometry>
    </collision>
    <inertial>
      <mass value="{self.__Robot_Properties['mass'][i]}"/>
      <origin rpy="0.0 0.0 0.0" xyz="{bbox_origin[0]:.05f} {bbox_origin[1]:.05f} {bbox_origin[2]:.05f}"/>
      <inertia ixx="{moi['I_xx']:.10f}" ixy="0.0" ixz="0.0" iyy="{moi['I_yy']:.10f}" iyz="0.0" izz="{moi['I_zz']:.10f}"/>
    </inertial>
  </link>'''
    
    def __Configure_Core(self, i: int, i_offset: int, T_i: tp.List[tp.List[float]], child_id: int, child_link: str, parent_link: str) -> str:
      # ...
      j_type = 'revolute' if self.__Robot_Str.Theta.Type[i] == 'R' else 'prismatic'
      j_ax = f'0 0 {int(self.__Robot_Str.Theta.Direction[i])}' if self.__Robot_Str.Theta.Axis[i] == 'Z' else f'{int(self.__Robot_Str.Theta.Direction[i])} 0 0'
      j_moi = MOI.Cube_MOI(self.__Robot_Properties['mass'][i + i_offset], self.__Robot_Str.Collider[i + i_offset].Size)
      j_bbox_origin = ((-1) *  self.__Robot_Str.Collider[i + i_offset].Origin) + [0.0, 0.0, 0.0]
      
      # ...
      obj_geometry = self.__Object_Geometry(f'Joint_{child_id}')

      # Get the translational and rotational part from the transformation matrix.
      p = ['0.0', '0.0', '0.0']; ea = ['0.0', '0.0', '0.0']
      for j, (p_i, ea_i) in enumerate(zip(np.round(T_i.p.all(), 5) + [0.0, 0.0, 0.0], 
                                          np.round(T_i.Get_Rotation('ZYX').all(), 5) + [0.0, 0.0, 0.0])):
        if p_i != 0.0:
            p[j] = f'{p_i:.05f}'
        if ea_i != 0.0:
            ea[j] = f'{ea_i:.10f}'

      return f'''  <!-- Configuration of the part called 'Joint {child_id}'. -->
  <joint name="joint_{child_id}" type="{j_type}">
    <parent link="{parent_link}"/>
    <child link="{child_link}"/>
    <origin rpy="{ea[0]} {ea[2]} {ea[1]}" xyz="{p[0]} {p[1]} {p[2]}"/>
    <axis xyz="{j_ax}"/>
    <limit effort="{self.__Robot_Properties['effort'][i]}" lower="{self.__Robot_Str.Theta.Limit[i, 0]:.10f}" upper="{self.__Robot_Str.Theta.Limit[i, 1]:.10f}" velocity="{self.__Robot_Properties['velocity'][i]}"/>
  </joint>
  <link name="{child_link}">
    <visual>
      <geometry>
        {obj_geometry['visual']}
      </geometry>
      <material name="Custom_Color">
        <color rgba="{self.__rgba}"/>
      </material>
    </visual>
    <collision>
      <geometry>
        {obj_geometry['collision']}
      </geometry>
    </collision>
    <inertial>
      <mass value="{self.__Robot_Properties['mass'][i + i_offset]}"/>
      <origin rpy="0.0 0.0 0.0" xyz="{j_bbox_origin[0]:.05f} {j_bbox_origin[1]:.05f} {j_bbox_origin[2]:.05f}"/>
      <inertia ixx="{j_moi['I_xx']:.10f}" ixy="0.0" ixz="0.0" iyy="{j_moi['I_yy']:.10f}" iyz="0.0" izz="{j_moi['I_zz']:.10f}"/>
    </inertial>
  </link>'''

    def __Configure_EE(self) -> str:
      return f'''  <!-- Configuration of the part called 'End-Effector (EE)'. -->
  <joint name="ee" type="fixed">
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
    <parent link="link_{len(self.__configuration['Core'])}"/>
    <child link="ee_link"/>
  </joint>
  <link name="ee_link"/>'''

    def Generate(self) -> None:
        print(f'[INFO] Generate a URDF file for the robot structure named {self.__Robot_Str.Name}.')
        print('[INFO] Identify links:')

        # ....
        self.__configuration['Base_0'] = self.__Configure_Base_0()

        """
        Description:
            Find the zero configuration of the homogeneous matrix of each joint using the modified 
            forward kinematics calculation method.
        """
        self.__Robot_Str.T.Zero_Cfg = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(self.__Robot_Str.Theta.Zero, 'Modified', 
                                                                                             self.__Robot_Str)[1]
        

        if self.__is_external_axis == True:
          # ..
          child_id = self.__Robot_Str.Theta.Name[0].removesuffix(f'_{self.__Robot_Str.Name}_ID_{self.__Robot_Str.Id:03}').removeprefix('Joint_')

          # ...
          self.__configuration['External'] = self.__Configure_Core(0, 1, self.__Robot_Str.T.Base.Inverse() @ self.__Robot_Str.T.Zero_Cfg[0], 
                                                                   child_id, f'link_{child_id}', 'base_link')
          print(f'[INFO] >> Index 0-0: Parent(base_link) -> Child(link_{child_id})')
          self.__configuration['Base_n'] = self.__Configure_Base_n(1, 2, 'base_link_1', f'link_{child_id}')
          print(f'[INFO] >> Index 0-1: Parent(link_{child_id}) -> Child(base_link_1)')

          # ...
          for i, T_Zero_i in enumerate(self.__Robot_Str.T.Zero_Cfg[1::], start=1):
            T_i = self.__Robot_Str.T.Zero_Cfg[i - 1].Inverse() @ T_Zero_i

            # ..
            child_id = self.__Robot_Str.Theta.Name[i].removesuffix(f'_{self.__Robot_Str.Name}_ID_{self.__Robot_Str.Id:03}').removeprefix('Joint_')
            # ..
            parent_link = 'base_link_1' if i == 1 else f'link_{i - 1}'

            # ..
            self.__configuration['Core'].append(self.__Configure_Core(i, 2, T_i, child_id, f'link_{child_id}', parent_link))
            print(f'[INFO] >> Index {i}: Parent({parent_link}) -> Child(link_{child_id})')

            # Release T_{i}.
            del T_i

        else:
          # ...
          for i, T_Zero_i in enumerate(self.__Robot_Str.T.Zero_Cfg):
            if i == 0:
              T_i = self.__Robot_Str.T.Base.Inverse() @ T_Zero_i
            else:
              T_i = self.__Robot_Str.T.Zero_Cfg[i - 1].Inverse() @ T_Zero_i

            # ..
            child_id = self.__Robot_Str.Theta.Name[i].removesuffix(f'_{self.__Robot_Str.Name}_ID_{self.__Robot_Str.Id:03}').removeprefix('Joint_')
            # ..
            parent_link = 'base_link' if i == 0 else f'link_{i}'

            # ..
            self.__configuration['Core'].append(self.__Configure_Core(i, 1, T_i, child_id, f'link_{child_id}', parent_link))
            print(f'[INFO] >> Index {i}: Parent({parent_link}) -> Child(link_{child_id})')

            # Release T_{i}.
            del T_i

        # ...
        self.__configuration['EE'] = self.__Configure_EE()
        print(f'[INFO] >> Index {self.__Robot_Str.Theta.Zero.shape[0]}: Parent(link_{child_id}) -> Child(ee_link)')
    
    def Save(self, file_path: str) -> None:
      """
      Description:
        ...
      
      Args:
        (1) file_path [string]: The specified path where the file should be saved.
                                Note:
                                    Whitout an extension '*.urdf'.
      """

      # Remove the '*.urdf' file if it already exists.
      if os.path.isfile(f'{file_path}.urdf'):
          os.remove(f'{file_path}.urdf')

      # Save the generated text to the '*.urdf' file.
      File_IO.Save(file_path, f'''<?xml version="1.0"?>\n<robot name="{self.__Robot_Str.Name}">''', 'urdf', '')
      File_IO.Save(file_path, self.__configuration['Base_0'], 'urdf', '')
      if self.__is_external_axis == True:
         File_IO.Save(file_path, self.__configuration['External'], 'urdf', '')
         File_IO.Save(file_path, self.__configuration['Base_n'], 'urdf', '')
      for _,  configuration_core_i in enumerate(self.__configuration['Core']):
        File_IO.Save(file_path, configuration_core_i, 'urdf', '')
      File_IO.Save(file_path, self.__configuration['EE'], 'urdf', '')
      File_IO.Save(file_path, '</robot>', 'urdf', '')

      # Display information.
      print(f'[INFO] The URDF configuration file was successfully saved to the folder:')
      print(f'[INFO] >> {file_path}.urdf')