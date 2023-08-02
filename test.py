import pybullet as p
import pybullet_data
import time
import numpy as np

# System (Default)
import sys
#   Add access if it is not in the system path.
if 'src' not in sys.path:
    sys.path.append('src/')
import Lib.Parameters.Robot as Robot_Param

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.setTimeStep(1.0/100.0)
p.setRealTimeSimulation(0)

base_id = p.loadURDF('URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True)

#this will disable all collisions with the cubeId
p.setCollisionFilterGroupMask(base_id, -1, 0, 0)

#this would enable/disable collision between a specific pair of objects
#enableCollision = 1
#p.setCollisionFilterPair(planeId, cubeId,-1,-1,enableCollision )

P1 = Robot_Param.ABB_IRB_14000_L_Str.T.Base.p
Q1 = Robot_Param.ABB_IRB_14000_L_Str.T.Base.Get_Rotation('QUATERNION')

P2 = Robot_Param.ABB_IRB_14000_R_Str.T.Base.p
Q2 = Robot_Param.ABB_IRB_14000_R_Str.T.Base.Get_Rotation('QUATERNION')

#quaternion: x, y, z, w
obj_id_1 = p.loadURDF('URDFs/Robots/ABB_IRB_14000_L/ABB_IRB_14000_L.urdf', [P1.x, P1.y, P1.z], [Q1.x, Q1.y, Q1.z, Q1.w], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#obj_id_2 = p.loadURDF('URDFs/Robots/ABB_IRB_14000_R/ABB_IRB_14000_R.urdf', [P2.x, P2.y, P2.z], [Q2.x, Q2.y, Q2.z, Q2.w], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)


p.setCollisionFilterPair(obj_id_1, base_id, -1,-1, 1)

while p.isConnected():
    p.stepSimulation()
    time.sleep(1.0/100.0)
