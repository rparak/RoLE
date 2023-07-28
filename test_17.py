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
urId = p.loadURDF('URDFs/Robots/Universal_Robots_UR3/Universal_Robots_UR3.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

jointPositions = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
index = 0
for j in range(p.getNumJoints(urId)):
    info = p.getJointInfo(urId, j)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_REVOLUTE):
        p.resetJointState(urId, j, jointPositions[index]) 
        p.setJointMotorControl2(urId, j, p.POSITION_CONTROL,
                                targetPosition=jointPositions[index],
                                force=100.0)
        index=index+1

#joint_orient = p.addUserDebugParameter("Joint", -180, 180, 0)

#p.loadURDF("URDFs/Viewpoint/Viewpoint.urdf", [0, 0, 0], useFixedBase=True, globalScaling=1.0)

# meters_per_second = degrees_per_second * (2 * pi * radius) / 360
    
current_ee_pos = p.getLinkState(urId, 6)
print(np.round(current_ee_pos[0], 3))

"""
joint_orient = p.addUserDebugParameter("Joint", -50, 50, 0)
while p.isConnected():
    p.setJointMotorControl2(urId, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(p.readUserDebugParameter(joint_orient)),
                            force=100.0)

    p.stepSimulation()
    time.sleep(1.0/100.0)
"""