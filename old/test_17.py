import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.setTimeStep(1.0/100.0)
p.setRealTimeSimulation(0)

# quaternion: x, y, z, w
#urId = p.loadURDF('URDFs/Robots/Universal_Robots_UR3/Universal_Robots_UR3.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#urId = p.loadURDF('URDFs/Robots/ABB_IRB_120/ABB_IRB_120.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#urId = p.loadURDF('URDFs/Robots/EPSON_LS3_B401S/EPSON_LS3_B401S.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
urId = p.loadURDF('URDFs/Robots/ABB_IRB_120_L_Ax/ABB_IRB_120_L_Ax.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

#urId = p.loadURDF('URDFs/Robots/ABB_IRB_120_L_Ax/ABB_IRB_120_L_Ax.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)


"""
jointPositions = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 0.0])
id = 0
for j in range(p.getNumJoints(urId)):
    info = p.getJointInfo(urId, j)
    jointName = info[1]
    jointType = info[2]
    if jointType in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        p.resetJointState(urId, j, jointPositions[id])
        id += 1 
        print(jointName, jointType)
"""

#joint_orient = p.addUserDebugParameter("Joint", -180, 180, 0)

#p.loadURDF("URDFs/Viewpoint/Viewpoint.urdf", [0, 0, 0], useFixedBase=True, globalScaling=1.0)

# meters_per_second = degrees_per_second * (2 * pi * radius) / 360
    
"""
current_ee_pos = p.getLinkState(urId, 6)
print(np.round(current_ee_pos[0], 3))
print(np.round(current_ee_pos[1], 3))

jP = p.calculateInverseKinematics(urId, 6, [0.302, 0.0, 0.358], current_ee_pos[1])
print(np.round(jP, 3))
"""

joint_orient = p.addUserDebugParameter("Joint", 0.0, 0.8, 0)
while p.isConnected():
    #p.setJointMotorControl2(urId, 3, p.POSITION_CONTROL, targetPosition=np.deg2rad(p.readUserDebugParameter(joint_orient)), force=100.0)
    p.setJointMotorControl2(urId, 0, p.POSITION_CONTROL, targetPosition=p.readUserDebugParameter(joint_orient), force=100.0)
    p.stepSimulation()
    time.sleep(1.0/100.0)
