import heapq
import numpy as np

def heuristic(a, b):
    return np.linalg.norm(np.array(a)-np.array(b))  # Euclidean

def neighbors(node, grid):
    x,y = node
    for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(1,-1),(-1,1)]:
        nx,ny = x+dx, y+dy
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
            if grid[nx,ny] == 0:  # 0 free, 1 occupied
                yield (nx,ny)

def proximity_cost(node, dist_map):
    # dist_map gives distance to nearest obstacle in meters
    d = dist_map[node]
    return 0.0 if d > 0.5 else (0.5 - d) * 10.0  # penalty for close proximity

def astar(start, goal, grid, dist_map):
    openq = []
    heapq.heappush(openq, (heuristic(start, goal), 0.0, start, None))
    came_from = {}
    gscore = {start: 0.0}
    while openq:
        f, g, current, parent = heapq.heappop(openq)
        if current in came_from: 
            continue
        came_from[current] = parent
        if current == goal:
            break
        for nbr in neighbors(current, grid):
            tentative_g = gscore[current] + np.linalg.norm(np.array(nbr)-np.array(current))
            tentative_g += proximity_cost(nbr, dist_map)  # add clearance penalty
            if tentative_g < gscore.get(nbr, np.inf):
                gscore[nbr] = tentative_g
                heapq.heappush(openq, (tentative_g + heuristic(nbr, goal), tentative_g, nbr, current))
    # reconstruct path
    path = []
    n = goal
    while n is not None:
        path.append(n)
        n = came_from.get(n)
    return path[::-1]
# grid and dist_map should be produced from sensor data / SLAM