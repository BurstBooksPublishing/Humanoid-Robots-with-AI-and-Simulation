import heapq
# node: (x,y,theta_index); map: occupancy grid + clearance grid
def heuristic(n, goal, alpha=1.0, beta=0.5):
    dx = (n[0]-goal[0]); dy = (n[1]-goal[1])
    dist = (dx*dx + dy*dy)**0.5
    dtheta = abs(((n[2]-goal[2]+3.14159) % (2*3.14159)) - 3.14159) # wrap
    return alpha*dist + beta*dtheta

def neighbors(node, grid, theta_steps=8):
    # generate neighbor poses: 8-connected with discrete theta changes
    x,y,t = node
    for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
        nx,ny = x+dx, y+dy
        if not grid.in_bounds(nx,ny): continue
        if grid.is_occupied(nx,ny): continue
        # allow small rotation change per move
        for dt in (-1,0,1):
            nt = (t+dt) % theta_steps
            yield (nx,ny,nt)

def edge_cost(u,v,clearance_grid, max_step=1.0):
    # distance cost
    dx = v[0]-u[0]; dy = v[1]-u[1]
    dist = (dx*dx+dy*dy)**0.5
    # clearance penalty (lower clearance -> higher penalty)
    clear_pen = max(0.0, (1.0 - clearance_grid[v[0],v[1]]))
    # turn penalty
    turn = abs(v[2]-u[2])
    return dist + 2.0*clear_pen + 0.2*turn

def astar(start, goal, grid, clearance_grid):
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    g = {start: 0.0}
    parent = {}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal: break
        for nbr in neighbors(current, grid):
            tentative = g[current] + edge_cost(current, nbr, clearance_grid)
            if tentative < g.get(nbr, float('inf')):
                g[nbr] = tentative
                f = tentative + heuristic(nbr, goal)
                heapq.heappush(open_set, (f, nbr))
                parent[nbr] = current
    # reconstruct path (omitted)
    return parent