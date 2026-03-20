import pybullet as p 
import time 

def visualize_trajectory(env, trajectory):
    drone = p.loadURDF("sphere2.urdf", [0, 0, 1])

    for point in trajectory:
        p.resetBasePositionAndOrientation(
            drone,
            [point[0], point[1], 1],
            [0, 0, 0, 1]
        )
        env.step()
        time.sleep(0.02)