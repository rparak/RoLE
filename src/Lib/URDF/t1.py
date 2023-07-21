# System (Default)
import sys
#   Add access if it is not in the system path.
sys.path.append('../..')
# OS (Operating system interfaces)
import os

import Lib.Utilities.File_IO as File_IO

urdf_head_configuration = '''<?xml version="1.0"?>
<robot name="Universal_Robots_UR3">'''

urdf_base_configuration_i = '''  <!-- Configuration of the part called 'Base 1'. -->
  <link name="Base_Link_1">
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
        <mesh filename="/Mesh/Visual/Base.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
'''

urdf_core_configuration_i = '''  <!-- Configuration of the part called 'Joint 1'. -->
  <joint name="Joint_1" type="revolute">
    <parent link="Base_Link_1"/>
    <child link="Link_1"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1519"/>
    <axis xyz="0 0 1"/>
    <limit effort="150.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="1.0"/>
  </joint>
  <link name="Link_1">
    <visual>
      <geometry>
        <mesh filename="/Mesh/Visual/Joint_1.stl"/>
      </geometry>
      <material name="Custom_Color">
        <color rgba="0.90 0.90 0.90 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <mesh filename="/Mesh/Visual/Joint_1.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>'''

urdf_ee_configuration = '''  <!-- Configuration of the part called 'End-Effector (EE)'. -->
  <joint name="EE" type="fixed">
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
    <parent link="Link_5"/>
    <child link="EE_Link"/>
  </joint>
  <link name="EE_Link"/>'''

urdf_last_configuration = '</robot>'

"""
print(urdf_head_configuration)
print(urdf_core_configuration_i)
print(urdf_ee_configuration)
print(urdf_last_configuration)
"""

file_path = 'output'
if os.path.isfile(file_path + '.urdf'):
    os.remove(file_path + '.urdf')
    print(f'[INFO] The file has been successfully removed.')
    print(f'[INFO]  - Path: {file_path}.urdf')


#import Core
#print(Core.Get_Physical_Properties('ABB_IRB_120'))
"""
File_IO.Save(file_path, urdf_head_configuration, 'urdf', '')
File_IO.Save(file_path, urdf_core_configuration_i, 'urdf', '')
File_IO.Save(file_path, urdf_ee_configuration, 'urdf', '')
File_IO.Save(file_path, urdf_last_configuration, 'urdf', '')
"""
