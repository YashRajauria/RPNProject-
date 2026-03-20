import numpy as np
import pybullet as p
import time 

from map.grid_map import GridMap
from planning.astar import astar
from trajectory.min_snap import minimum_snap_2d
from planning.corridor import generate_corridor
from simulation.quadrotor_env import QuadrotorEnv
from trajectory.sampler import trajectory_sample


# PATH REDUCTION
def reduce_path(path, angle_thresh=np.deg2rad(10)):
    reduced = [path[0]]

    for i in range(1, len(path)-1):
        p_prev = np.array(path[i-1], dtype=float)
        p_curr = np.array(path[i], dtype= float)
        p_next = np.array(path[i+1], dtype=float)

        v1 = p_curr - p_prev
        v2 = p_next - p_curr

        v1 /= (np.linalg.norm(v1) + 1e-6)
        v2 /= (np.linalg.norm(v2) + 1e-6)

        angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

        if angle > angle_thresh:
            reduced.append(path[i])

    reduced.append(path[-1])
    return reduced


# TIME ALLOCATION (FIXED)
def compute_cumulative_times(path, v_max=5.0):
    times = [0.0]

    for i in range(len(path)-1):
        dist = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))

        # prevent zero-time segments
        dt = max(dist / v_max, 1.2)

        times.append(times[-1] + dt)

    return times


# VISUALIZATION
def visualize_obstacles(grid):
    for (xmin, ymin), (xmax, ymax) in grid.obstacles:

        center = [(xmin + xmax)/2, (ymin + ymax)/2, 0.5]
        size = [(xmax - xmin)/2, (ymax - ymin)/2, 0.5]

        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=size,
            rgbaColor=[1, 0, 0, 0.8]
        )

        p.createMultiBody(0, col, vis, center)


def visualize_corridors_3d(corridors):
    for (xmin, xmax, ymin, ymax) in corridors:
        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2

        sx = (xmax - xmin) / 2
        sy = (ymax - ymin) / 2
        sz = 2.0

        visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[sx, sy, sz],
            rgbaColor=[0, 1, 0, 0.25]
        )

        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual,
            basePosition=[cx, cy, sz]
        )


def visualize_trajectory(px, py, times):
    pts = []

    for i in range(px.shape[0]):
        for tau in np.linspace(0,1,60):

            basis = np.array([tau**k for k in range(8)])

            x = px[i] @ basis
            y = py[i] @ basis

            pts.append([x,y, 1.0])
    
    for i in range(len(pts)-1):
        p.addUserDebugLine(pts[i], pts[i+1], [0, 0, 1], 2)
        


# MAP SETUP
grid = GridMap(55, 55)

#Random Obstacles
num_obstacles = 40

for _ in range(num_obstacles):
    x = np.random.randint(0, 45)
    y = np.random.randint(0, 45)

    w = np.random.randint(2, 4)
    h = np.random.randint(2, 4)

    if 23 < x < 28:
        continue

    grid.add_obstacle_rect((x,y), (x+w,y+h))

#Structured narrow passage 
for y in range(0,50):
    if y not in range(20,30): 
        grid.add_obstacle_rect((25,y), (26, y+1))

for x in range(0,50):
    if x not in range(15,25):
        grid.add_obstacle_rect((x, 30), (x+1, 31))

#Clustered Obstacles 
for _ in range(5):
    cx = np.random.randint(10, 40)
    cy = np.random.randint(10, 40)

    for _ in range(3):
        dx = np.random.randint(-2,2)
        dy = np.random.randint(-2,2)

        grid.add_obstacle_rect(
            (cx+dx, cy+dy),
            (cx+dx+2, cy+dy+2)
        )

start = (3,3)
goal = (50,50)


# PATH PLANNING
path = astar(grid, start, goal)

if path is None:
    raise RuntimeError("A* failed!")

path = reduce_path(path)
print(f"[INFO] Reduced path length: {len(path)}")



# CORRIDORS
corridors = generate_corridor(grid, path, inflation=2)

if len(corridors) == len(path):
    corridors = corridors[:-1]

print(f"[DEBUG] Corridors: {len(corridors)}")



# TIME 
times = compute_cumulative_times(path)

print(f"[DEBUG] Total trajectory time: {times[-1]:.2f}s")


# MIN SNAP
px, py = minimum_snap_2d(path, times, corridors)


# SIMULATION
env = QuadrotorEnv(gui=True)

p.resetDebugVisualizerCamera(
    cameraDistance=8,
    cameraYaw=60,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0]
)

visualize_obstacles(grid)
visualize_corridors_3d(corridors)
visualize_trajectory(px, py, times)


# CONTROL LOOP
t = 0
dt = 0.01
paused = False

print("[INFO] Controls:")
print("SPACE → Pause/Resume | R → Reset | ESC → Exit")

while p.isConnected():

    keys = p.getKeyboardEvents()

    if 65307 in keys:
        break

    if ord(' ') in keys:
        paused = not paused
        time.sleep(0.2)

    if ord('r') in keys:
        t = 0
        print("[INFO] Reset")
        time.sleep(0.2)

    if not paused:

        if t <= times[-1]:  

            x_des, y_des = trajectory_sample(px, py, times, t)
            z_des = 1.0

            pos, vel = env.get_state()

            target = np.array([x_des, y_des, z_des])
            error = target - pos
            derror = -vel

            kp = np.array([8, 8, 15])
            kd = np.array([5, 5, 8])

            acc_cmd = kp * error + kd * derror

            force = np.array([
                acc_cmd[0],
                acc_cmd[1],
                9.81 + acc_cmd[2]
            ])

            env.apply_control(force)

            t += dt

    env.step(dt)
    time.sleep(dt)