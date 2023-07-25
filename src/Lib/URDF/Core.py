# Typing (Support for type hints)
import typing as tp
# OS (Operating system interfaces)
import os
# Custom Script:
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core
# Custom Script:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Utilities/File_IO
import Lib.Utilities.File_IO as File_IO
#   ../Lib/Utilities/MOI
import Lib.Utilities.MOI as MOI

def Get_Physical_Properties(name: str) -> tp.Tuple[float]:
    """
    Description:
        Get the defined physical properties of the robotic structure.

            Properties:
                Mass:
                    Weight of individual robot parts in kilograms.
                Effort:
                    An attribute for enforcing the maximum joint effort.
                Velocity:
                    An attribute for enforcing the maximum joint velocity.

          Warning:
            Please note that these values are approximate.
                
    Args:
        (1) name [string]: Name of the robotic structure.

    Returns:
        (1) parameter [Dictionary {'mass', Vector<float> 1xn, etc.}]: Defined physical properties of the robotic 
                                                                      structure.
                                                                        Note:
                                                                            Where n is the number of joints + base.
    """

    return {
        'Universal_Robots_UR3': {'mass': [2.0, 2.0, 3.42, 1.26, 0.8, 0.8, 0.35], 
                                 'effort': [56.0, 56.0, 28.0, 12.0, 12.0, 12.0], 
                                 'velocity': [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]},
        'ABB_IRB_120': {'mass': [8.25, 3.45, 5.9, 2.95, 2.30, 1.55, 0.60], 
                        'effort': [88.0, 88.0, 44.0, 22.0, 22.0, 22.0], 
                        'velocity': [4.36, 4.36, 4.36, 5.58, 5.58, 7.33]},
        'ABB_IRB_120_L_Ax': {'mass': [25.0, 8.25, 3.45, 5.9, 2.95, 2.30, 1.55, 0.60], 
                             'effort': [4.0, 2.0, 2.0, 1.0, 0.5, 0.5, 0.5], 
                             'velocity': [0.5, 0.5, 0.5, 0.5, 0.75, 0.75, 1.0]},
        'ABB_IRB_14000_R': {'mass': [24.0, 2.0, 2.5, 2.0, 2.0, 2.0, 0.5],
                            'effort': [36.0, 36.0, 12.0, 12.0, 6.0, 6.0, 6.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'ABB_IRB_14000_L': {'mass': [24.0, 2.0, 2.5, 2.0, 2.0, 2.0, 0.5], 
                            'effort': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'velocity': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
        'EPSON_LS3_B401S': {'mass': [5.5, 3.0, 4.5, 0.5, 0.5], 
                            'effort': [88.0, 88.0, 44.0, 22.0], 
                            'velocity': [6.28, 6.28, 1.0, 12.56]}
    }[name]

def Generate_URDF(Robot_Str: Parameters.Robot_Parameters_Str, file_path: str) -> None:
    """
    Description:
        ...

    Args:
        (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (2) file_path [string]: The specified path where the file should be saved.
                                Note:
                                    Whitout an extension '*.urdf'.
    """

    # Get the defined physical properties of the robotic structure.
    Robot_Physical_Properties = Get_Physical_Properties(Robot_Str.Name)

    # ...
    urdf_head_configuration = f'''<?xml version="1.0"?>\n<robot name="{Robot_Str.Name}">'''

    # ...
    base_moi = MOI.Cube_MOI(Robot_Physical_Properties['mass'][0], Robot_Str.Collider[0].Size)

    # ...
    urdf_base_configuration = f'''  <!-- Configuration of the part called 'Base 1'. -->
  <link name="Base_Link">
    <visual>
      <geometry>
        <mesh filename="/Mesh/Visual/Base.stl"/>
      </geometry>
      <material name="Custom_Color">
        <color rgba="0.90 0.90 0.90 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <mesh filename="/Mesh/Collision/Base.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="{Robot_Physical_Properties['mass'][0]}"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <inertia ixx="{base_moi['I_xx']:.10f}" ixy="0.0" ixz="0.0" iyy="{base_moi['I_yy']:.10f}" iyz="0.0" izz="{base_moi['I_zz']:.10f}"/>
    </inertial>
  </link>'''

    """
    Description:
        Find the zero configuration of the homogeneous matrix of each joint using the modified 
        forward kinematics calculation method.
    """
    Robot_Str.T.Zero_Cfg = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(Robot_Str.Theta.Zero, 'Modified', Robot_Str)[1]

    urdf_core_configuration = []
    for i in range(Robot_Str.Theta.Zero.shape[0]):
        if i == 0:
            parent_str = f'Base_Link_{i + 1}'
          
            # ...
            T_i = Robot_Str.T.Base.Inverse() @ Robot_Str.T.Zero_Cfg[i]
        else:
            parent_str = f'Link_{i}'

            # ...
            T_i = Robot_Str.T.Zero_Cfg[i - 1].Inverse() @ Robot_Str.T.Zero_Cfg[i]

        # ...
        #   ...
        joint_id = Robot_Str.Theta.Name[i].removesuffix(f'_{Robot_Str.Name}_ID_{Robot_Str.Id:03}').removeprefix('Joint_')
        child_str = f'Link_{joint_id}'
        #   ...
        joint_type = 'revolute' if Robot_Str.Theta.Type[i] == 'R' else 'prismatic'
        #   ...
        if Robot_Str.Theta.Axis[i] == 'Z':
          # ...
          joint_axis = f'0 0 {int(Robot_Str.Theta.Direction[i])}'

          # ...
          joint_moi = MOI.Cube_MOI(Robot_Physical_Properties['mass'][i + 1], [Robot_Str.Collider[i + 1].Size[2],
                                                                              Robot_Str.Collider[i + 1].Size[1],
                                                                              Robot_Str.Collider[i + 1].Size[0]])
        else:
          # ...
          joint_axis = f'{int(Robot_Str.Theta.Direction[i])} 0 0'

          # ...
          joint_moi = MOI.Cube_MOI(Robot_Physical_Properties['mass'][i + 1], Robot_Str.Collider[i + 1].Size)
        
        # Get the translational and rotational part from the transformation matrix.
        p = T_i.p; Euler_Angles = T_i.Get_Rotation('ZYX') + [0.0, 0.0, 0.0]

        # ...
        urdf_core_configuration.append(f'''  <!-- Configuration of the part called 'Joint {joint_id}'. -->
  <joint name="Joint_{joint_id}" type="{joint_type}">
    <parent link="{parent_str}"/>
    <child link="{child_str}"/>
    <origin rpy="{Euler_Angles.x:.10f} {Euler_Angles.y:.10f} {Euler_Angles.z:.10f}" xyz="{p.x:.05f} {p.y:.05f} {p.z:.05f}"/>
    <axis xyz="{joint_axis}"/>
    <limit effort="{Robot_Physical_Properties['effort'][i]}" lower="{Robot_Str.Theta.Limit[i, 0]:.10f}" upper="{Robot_Str.Theta.Limit[i, 1]:.10f}" velocity="{Robot_Physical_Properties['velocity'][i]}"/>
  </joint>
  <link name="Link_{joint_id}">
    <visual>
      <geometry>
        <mesh filename="/Mesh/Visual/Joint_{joint_id}.stl"/>
      </geometry>
      <material name="Custom_Color">
        <color rgba="0.90 0.90 0.90 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <mesh filename="/Mesh/Collision/Joint_{joint_id}.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="{Robot_Physical_Properties['mass'][i + 1]}"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <inertia ixx="{joint_moi['I_xx']:.10f}" ixy="0.0" ixz="0.0" iyy="{joint_moi['I_yy']:.10f}" iyz="0.0" izz="{joint_moi['I_zz']:.10f}"/>
    </inertial>
  </link>''')
        
        # Release T_{i}.
        del T_i

    # ...
    urdf_ee_configuration = f'''  <!-- Configuration of the part called 'End-Effector (EE)'. -->
  <joint name="EE" type="fixed">
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
    <parent link="Link_{i + 1}"/>
    <child link="EE_Link"/>
  </joint>
  <link name="EE_Link"/>
  </link>'''

    # ...
    urdf_last_configuration = '</robot>'

    # Remove the '*.urdf' file if it already exists.
    if os.path.isfile(f'{file_path}.urdf'):
        os.remove(f'{file_path}.urdf')

    # Save the generated text to the '*.urdf' file.
    File_IO.Save(file_path, urdf_head_configuration, 'urdf', '')
    File_IO.Save(file_path, urdf_base_configuration, 'urdf', '')
    for _,  urdf_core_configuration_i in enumerate(urdf_core_configuration):
        File_IO.Save(file_path, urdf_core_configuration_i, 'urdf', '')
    File_IO.Save(file_path, urdf_ee_configuration, 'urdf', '')
    File_IO.Save(file_path, urdf_last_configuration, 'urdf', '')
