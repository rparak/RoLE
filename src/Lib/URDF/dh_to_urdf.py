import xml.etree.ElementTree as ET

def ur3_dh_parameters_to_urdf():
    dh_parameters = [
        [       0,  1.57079632679,  0.1519, 0],
        [-0.24365,            0.0,       0, 0],
        [-0.21325,              0,       0, 0],
        [       0,  1.57079632679, 0.11235, 0],
        [       0, -1.57079632679, 0.08535, 0],
        [       0,            0.0,  0.0819, 0]
    ]

    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    link_names = ["base_link", "shoulder_link", "upper_arm_link",
                  "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]

    robot_name = "ur3_robot"

    urdf_text = f'<?xml version="1.0"?>\n<robot name="{robot_name}">\n\n'

    for idx, dh_param in enumerate(dh_parameters):
        a, alpha, d, theta = dh_param
        joint_name = joint_names[idx]
        link_name = link_names[idx + 1] if idx < len(link_names) - 1 else "end_effector_link"

        urdf_text += f'  <joint name="{joint_name}" type="revolute">\n'
        urdf_text += f'    <origin xyz="{a} 0 {d}" rpy="0 0 {alpha}" />\n'
        urdf_text += f'    <parent link="{link_names[idx]}" />\n'
        urdf_text += f'    <child link="{link_name}" />\n'
        urdf_text += f'    <axis xyz="0 0 1" />\n'
        urdf_text += f'    <limit effort="100" lower="{theta}" upper="{theta}" velocity="3.14" />\n'
        urdf_text += f'  </joint>\n\n'

        urdf_text += f'  <link name="{link_name}">\n'
        urdf_text += f'    <visual>\n'
        urdf_text += f'      <origin xyz="0 0 0" rpy="0 0 0" />\n'
        urdf_text += f'      <geometry>\n'
        urdf_text += f'        <box size="0.1 0.1 0.1" />\n'
        urdf_text += f'      </geometry>\n'
        urdf_text += f'    </visual>\n'
        urdf_text += f'    <collision>\n'
        urdf_text += f'      <origin xyz="0 0 0" rpy="0 0 0" />\n'
        urdf_text += f'      <geometry>\n'
        urdf_text += f'        <box size="0.1 0.1 0.1" />\n'
        urdf_text += f'      </geometry>\n'
        urdf_text += f'    </collision>\n'
        urdf_text += f'  </link>\n\n'

    urdf_text += '</robot>'
    return urdf_text

if __name__ == "__main__":
    urdf_xml = ur3_dh_parameters_to_urdf()
    print(urdf_xml)
