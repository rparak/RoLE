import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.setTimeStep(1.0/100.0)
p.setRealTimeSimulation(0)

obj_id = p.loadURDF('URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf', [0.0, 0.0, 0.0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
while p.isConnected():
    p.stepSimulation()
    time.sleep(1.0/100.0)
