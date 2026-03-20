import heapq

def astar(grid_map, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    cost = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            break

        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            neighbor = (current[0]+dx, current[1]+dy)

            if not grid_map.is_free(neighbor[0], neighbor[1]):
                continue

            new_cost = cost[current]+1

            if neighbor not in cost or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost

                priority = new_cost + abs(goal[0] - neighbor[0]) + abs(goal[1] - neighbor[1])

                heapq.heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current



    path = [] #Path Reconstruction 
    cur = goal

    if cur not in came_from:
        raise ValueError("No path found!")
    
    while cur != start:
        path.append(cur)
        cur = came_from[cur]

    path.append(start)
    path.reverse()

    return path 
