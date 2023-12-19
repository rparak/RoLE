# Typing (Support for type hints)
import typing as tp
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../Lib/Kinematics/Core
import RoLE.Kinematics.Core
#   ../RoLE/URDF/Utilities
import RoLE.URDF.Utilities as Utilities
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO as File_IO
#   ../RoLE/Utilities/MOI
import RoLE.Utilities.MOI as MOI

class URDF_Generator_Cls(object):
    """
    Description:
      A specific class for working with URDFs (Unified Robotics Description Format).

      Reference:
        http://wiki.ros.org/urdf/XML

    Initialization of the Class:
        Args:
          (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
          (2) use_mesh [bool]: Use a mesh to represent a visual/collision object. Otherwise, use a sphere to 
                              represent the object.
          (3) rgba [Vector<float> 1x4]: The colour of the material determined by a set of four numbers representing 
                                        rgba(red, green, blue and alpha), each in the range (0 - 1).

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the ABB IRB 120 robot.
                Robot_Parameters_Str = RoLE.Parameters.Robot.ABB_IRB_120_Str
                use_mesh = True

                # Initialization of the class.
                Cls = URDF_Generator_Cls(Robot_parameters_Str, use_mesh)

            Features:
                # Functions of the class.
                Cls.Generate(); Cls.Save()
    """

    def __init__(self, Robot_Parameters_Str: Parameters.Robot_Parameters_Str, use_mesh: bool, rgba: str) -> None:
        # Express the class input parameters as private variables.
        self.__Robot_Parameters_Str = Robot_Parameters_Str
        self.__use_mesh = use_mesh
        # Convert the material parameter to a string without a comma.
        self.__rgba = ' '.join(str(x) for x in rgba)

        # Get the defined physical properties of the robotic structure.
        self.__Robot_Properties = Utilities.Get_Physical_Properties(self.__Robot_Parameters_Str.Name)

        # Create a configuration dictionary depending on the type of robotic structure.
        if self.__Robot_Parameters_Str.External_Axis == True:
          self.__configuration = {'Base_0': '', 'External': '', 'Base_n': '', 'Core': [], 'EE': ''}
        else:
          self.__configuration = {'Base_0': '', 'Core': [], 'EE': ''}
    
    def __Object_Geometry(self, name: str) -> tp.Dict[str, str]:
      """
      Description:
        Get the visual/collision geometry of an individual part of the robotic structure in the 
        form of a string.
      
      Args:
        (1) name [string]: The name of the geometric part of the link.
                           Note:
                            Only if mesh use is enabled.

      Returns:
        (1) parameter [Dictonary(string, string)]: Visual/collision geometry of an individual part 
                                                   of the robotic structure in the form of a string.
      """
            
      if self.__use_mesh == True:
          return {'visual': f'<mesh filename="/Mesh/Visual/{name}.stl"/>', 
                  'collision': f'<mesh filename="/Mesh/Visual/{name}.stl"/>'}
      else:
          return {'visual': '<sphere radius="0.01"/>', 
                  'collision': '<sphere radius="0.01"/>'}
      
    def __Configure_Base_0(self) -> str:
      """
      Description:
        Get the URDF configuration of the base (id - 0) of the robotic structure.

      Returns:
        (1) parameter [string]: The URDF configuration of the base (id - 0) of the robotic structure.
      """
            
      # Get the visual/collision geometry of an individual part of the robotic structure.
      obj_geometry = self.__Object_Geometry('Base')

      # Get a list of base colliders.
      Base_Collider = list(self.__Robot_Parameters_Str.Collider.Base.values())

      # Obtain the moment of inertia (MOI) for the bounding box.
      moi = MOI.Cube(self.__Robot_Properties['mass'][0], Base_Collider[0].Size)

      # Change the direction of the origin of the bounding box.
      bbox_origin = ((-1) *  Base_Collider[0].Origin) + [0.0, 0.0, 0.0]

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

    def __Configure_Base_n(self, n: int, i: int, child_link: str, parent_link: str) -> str:
      """
      Description:
        Get the URDF configuration of the base (id - n) of the robotic structure.
      
      Args:
        (1) n [int]: The identification number of the base fixed joint.
        (2) i [int]: Iteration.
        (3) child_link [string]: The name of the child link.
        (4) parent_link [string]: The name of the parent link.

      Returns:
        (1) parameter [string]: The URDF configuration of the base (id - n) of the robotic structure.
      """
            
      # Get a list of base colliders.
      Base_Collider = list(self.__Robot_Parameters_Str.Collider.Base.values())

      # Get the visual/collision geometry of an individual part of the robotic structure.
      obj_geometry = self.__Object_Geometry(f'Base_{n}')

      # Obtain the moment of inertia (MOI) for the bounding box.
      moi = MOI.Cube(self.__Robot_Properties['mass'][i], Base_Collider[-1].Size)

      # Change the direction of the origin of the bounding box.
      bbox_origin = ((-1) *  Base_Collider[-1].Origin) + [0.0, 0.0, 0.0]

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
      """
      Description:
        Get the URDF configuration of the joint parts of the robotic structure.
      
      Args:
        (1) i [int]: Interation.
        (2) i_offset [int]: Iteration offset.
        (3) T_i [Matrix<float> 4x4]: Homogeneous transformation matrix in episode i.
        (4) child_id [string]: The id of the child.
        (4) child_link [string]: The name of the child link.
        (5) parent_link [string]: The name of the parent link.

      Returns:
        (1) parameter [string]: The URDF configuration of the joint parts of the robotic structure.
      """

      # Get a list of joint colliders.
      Theta_Collider = list(self.__Robot_Parameters_Str.Collider.Theta.values())

      # Get the main parameters of the joint to create the configuration.
      #   The type of joint (prismatic, revolute).
      j_type = 'revolute' if self.__Robot_Parameters_Str.Theta.Type[i] == 'R' else 'prismatic'
      #   The joint axis specified in the joint frame. 
      j_ax = f'0 0 {int(self.__Robot_Parameters_Str.Theta.Direction[i])}' if self.__Robot_Parameters_Str.Theta.Axis[i] == 'Z' else f'{int(self.__Robot_Parameters_Str.Theta.Direction[i])} 0 0'
      #   Moment of inertia (MOI) of the bounding box.
      j_moi = MOI.Cube(self.__Robot_Properties['mass'][i + i_offset], Theta_Collider[i + i_offset].Size)
      #   Origin of the bounding box..
      j_bbox_origin = ((-1) *  Theta_Collider[i + i_offset].Origin) + [0.0, 0.0, 0.0]
      
      # Get the visual/collision geometry of an individual part of the robotic structure.
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
    <limit effort="{self.__Robot_Properties['effort'][i]}" lower="{self.__Robot_Parameters_Str.Theta.Limit[i, 0]:.10f}" upper="{self.__Robot_Parameters_Str.Theta.Limit[i, 1]:.10f}" velocity="{self.__Robot_Properties['velocity'][i]}"/>
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
      """
      Description:
        Get the URDF configuration of the end-effector.

      Returns:
        (1) parameter [string]: The URDF end-effector configuration.
      """

      return f'''  <!-- Configuration of the part called 'End-Effector (EE)'. -->
  <joint name="ee" type="fixed">
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
    <parent link="link_{len(self.__configuration['Core'])}"/>
    <child link="ee_link"/>
  </joint>
  <link name="ee_link"/>'''

    def Generate(self) -> None:
        """
        Description:
          Generate a URDF file for the robotic structure that is defined in the class input parameters.
        """

        print(f'[INFO] Generate a URDF file for the robot structure named {self.__Robot_Parameters_Str.Name}.')
        print('[INFO] Identify links:')

        # Get the URDF configuration of the base (id - 0) of the robotic structure.
        self.__configuration['Base_0'] = self.__Configure_Base_0()

        # Find the zero configuration of the homogeneous transformation matrix of each joint using the modified 
        # forward kinematics calculation method.
        self.__Robot_Parameters_Str.T.Zero_Cfg = RoLE.Kinematics.Core.Get_Individual_Joint_Configuration(self.__Robot_Parameters_Str.Theta.Zero, 'Modified', 
                                                                                                        self.__Robot_Parameters_Str)[1]
        
        # Generate a configuration for the URDF file with individual principles that depend on whether 
        # the robot contains an external axis or not.
        if self.__Robot_Parameters_Str.External_Axis == True:
          # Get the child's ID from the string.
          child_id = self.__Robot_Parameters_Str.Theta.Name[0].removesuffix(f'_{self.__Robot_Parameters_Str.Name}_ID_{self.__Robot_Parameters_Str.Id:03}').removeprefix('Joint_')

          # Get the URDF configuration of the external joint.
          self.__configuration['External'] = self.__Configure_Core(0, 1, self.__Robot_Parameters_Str.T.Base.Inverse() @ self.__Robot_Parameters_Str.T.Zero_Cfg[0], 
                                                                   child_id, f'link_{child_id}', 'base_link')
          print(f'[INFO] >> Index 0-0: Parent(base_link) -> Child(link_{child_id})')
          
          # Get the URDF configuration of the base (id - n) of the robotic structure.
          self.__configuration['Base_n'] = self.__Configure_Base_n(1, 2, 'base_link_1', f'link_{child_id}')
          print(f'[INFO] >> Index 0-1: Parent(link_{child_id}) -> Child(base_link_1)')

          for i, T_Zero_i in enumerate(self.__Robot_Parameters_Str.T.Zero_Cfg[1::], start=1):
            # Equation to obtain the transformation matrix between two parts.
            T_i = self.__Robot_Parameters_Str.T.Zero_Cfg[i - 1].Inverse() @ T_Zero_i

            # Get the child's ID from the string.
            child_id = self.__Robot_Parameters_Str.Theta.Name[i].removesuffix(f'_{self.__Robot_Parameters_Str.Name}_ID_{self.__Robot_Parameters_Str.Id:03}').removeprefix('Joint_')
            
            # Express the parent link.
            parent_link = 'base_link_1' if i == 1 else f'link_{child_id_last}'

            # Get the URDF configuration of the joint in episode (i).

            self.__configuration['Core'].append(self.__Configure_Core(i, 0, T_i, child_id, f'link_{child_id}', parent_link))
            print(f'[INFO] >> Index {i}: Parent({parent_link}) -> Child(link_{child_id})')

            # Save the child's last identifier.
            child_id_last = child_id

            # Release T_{i}.
            del T_i

        else:
          for i, T_Zero_i in enumerate(self.__Robot_Parameters_Str.T.Zero_Cfg):
            # Equation to obtain the transformation matrix between two parts.
            if i == 0:
              T_i = self.__Robot_Parameters_Str.T.Base.Inverse() @ T_Zero_i
            else:
              T_i = self.__Robot_Parameters_Str.T.Zero_Cfg[i - 1].Inverse() @ T_Zero_i

            # Get the child's ID from the string.
            child_id = self.__Robot_Parameters_Str.Theta.Name[i].removesuffix(f'_{self.__Robot_Parameters_Str.Name}_ID_{self.__Robot_Parameters_Str.Id:03}').removeprefix('Joint_')
            
            # Express the parent link.
            parent_link = 'base_link' if i == 0 else f'link_{child_id_last}'

            # Get the URDF configuration of the joint in episode (i).
            self.__configuration['Core'].append(self.__Configure_Core(i, 0, T_i, child_id, f'link_{child_id}', parent_link))
            print(f'[INFO] >> Index {i}: Parent({parent_link}) -> Child(link_{child_id})')

            # Save the child's last identifier.
            child_id_last = child_id

            # Release T_{i}.
            del T_i

        # Get the URDF configuration of the end-effector.
        self.__configuration['EE'] = self.__Configure_EE()
        print(f'[INFO] >> Index {self.__Robot_Parameters_Str.Theta.Zero.shape[0]}: Parent(link_{child_id}) -> Child(ee_link)')
    
    def Save(self, file_path: str) -> None:
      """
      Description:
        Function to save URDF configurations to a file.
      
      Args:
        (1) file_path [string]: The specified path where the file should be saved.
                                Note:
                                    Whitout an extension '*.urdf'.
      """

      # Remove the '*.urdf' file if it already exists.
      if os.path.isfile(f'{file_path}.urdf'):
          os.remove(f'{file_path}.urdf')

      # Save the generated text to the '*.urdf' file.
      File_IO.Save(file_path, f'''<?xml version="1.0"?>\n<robot name="{self.__Robot_Parameters_Str.Name}">''', 'urdf', '')
      File_IO.Save(file_path, self.__configuration['Base_0'], 'urdf', '')
      if self.__Robot_Parameters_Str.External_Axis == True:
         File_IO.Save(file_path, self.__configuration['External'], 'urdf', '')
         File_IO.Save(file_path, self.__configuration['Base_n'], 'urdf', '')
      for _,  configuration_core_i in enumerate(self.__configuration['Core']):
        File_IO.Save(file_path, configuration_core_i, 'urdf', '')
      File_IO.Save(file_path, self.__configuration['EE'], 'urdf', '')
      File_IO.Save(file_path, '</robot>', 'urdf', '')

      # Display information.
      print(f'[INFO] The URDF configuration file was successfully saved to the folder:')
      print(f'[INFO] >> {file_path}.urdf')