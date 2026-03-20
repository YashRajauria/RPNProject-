import numpy as np 

def compute_segment_times(waypoints, v_max=2.0, a_max=2.0):

    times = []
    
    for i in range(len(waypoints)-1):
        p0 = np.array(waypoints[i])
        p1 = np.array(waypoints[i+1])

        dist = np.linalg.norm(p1-p0)

        T = dist / v_max #Basic constant velocity model

        T = max(T, 0.5) #Add safety margin 
        times.append(T)

    return times 