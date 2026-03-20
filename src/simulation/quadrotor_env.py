import pybullet as p 
import pybullet_data 
import numpy as np 
import time 

class QuadrotorEnv:
    def __init__(self, gui=True):
        self.gui = gui

        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self._setup_world()
        self._create_drone()

    def _setup_world(self):
        self.plane = p.loadURDF("plane.urdf")

        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=6,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0]
            )

    def _create_drone(self):
        mass = 1.0 # Quadrator drone 
        body = p.createVisualShape(p.GEOM_SPHERE, radius=0.1)
        self.drone = p.createMultiBody(
            baseMass = mass,
            baseCollisionShapeIndex=body,
            basePosition=[0, 0, 1]
        )

        arm1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.02, 0.02])
        arm2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.3, 0.02])

        p.createMultiBody(baseMass=0, baseVisualShapeIndex=arm1, basePosition=[0, 0, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=arm2, basePosition=[0, 0, 1])

        for dx, dy in [(0.3,0), (-0.3,0), (0,0.3), (0,-0.3)]:
            rotor = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.01)
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=rotor,
                basePosition=[dx, dy, 1]
            )


    def get_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.drone)
        vel, ang_vel = p.getBaseVelocity(self.drone)
        return np.array(pos), np.array(vel)
    
    def apply_control(self, force):
        #thrust = total upward force
        force = np.array(force, dtype=float)
        force[2] += 9.81
        
        p.applyExternalForce(
            objectUniqueId=self.drone,
            linkIndex=-1,
            forceObj=force.tolist(),
            posObj=[0, 0, 0],
            flags=p.WORLD_FRAME
        )
    
    def step(self, dt=0.01):
        p.stepSimulation()
        time.sleep(dt)

    def reset(self):
        p.resetBasePositionAndOrientation(
            self.drone,
            [0, 0, 1],
            [0, 0, 0, 1]
        )
        
        p.resetBaseVelocity(
            self.drone,
            [0, 0, 0],
            [0, 0, 0]
        )

    def close(self):
        p.disconnect()