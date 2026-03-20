import pybullet as p
import pybullet_data
import time

class PyBulletEnv:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

    def step(self):
        p.stepSimulation()
        time.sleep(1./240.)

    def close(self):
        p.disconnect()