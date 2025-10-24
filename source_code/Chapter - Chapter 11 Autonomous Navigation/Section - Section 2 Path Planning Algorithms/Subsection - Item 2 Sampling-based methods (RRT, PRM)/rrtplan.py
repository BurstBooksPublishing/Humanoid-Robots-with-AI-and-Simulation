import random, math
# Node class
class Node:
    def __init__(self, q, parent=None):
        self.q = q          # 3-DoF pose: (x,y,theta)
        self.parent = parent

def distance(a,b):
    dx,dy = a[0]-b[0], a[1]-b[1]
    dtheta = math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2]))
    return math.hypot(dx,dy) + 0.1*abs(dtheta)  # weighted metric

def sample_random(bounds):
    x = random.uniform(bounds['xmin'], bounds['xmax'])
    y = random.uniform(bounds['ymin'], bounds['ymax'])
    theta = random.uniform(-math.pi, math.pi)
    return (x,y,theta)

def nearest(tree, q_rand):
    return min(tree, key=lambda n: distance(n.q, q_rand))

def steer(q_from, q_to, step=0.2):
    # linear interpolation in SE(2) by fixed step length
    dist = distance(q_from, q_to)
    if dist <= step: return q_to
    alpha = step/dist
    x = q_from[0] + alpha*(q_to[0]-q_from[0])
    y = q_from[1] + alpha*(q_to[1]-q_from[1])
    theta = math.atan2(math.sin((1-alpha)*q_from[2]+alpha*q_to[2]), 1)  # approx
    return (x,y,theta)

def plan_rrt(q_start, q_goal, bounds, max_iter=5000):
    tree = [Node(q_start)]
    for _ in range(max_iter):
        q_rand = sample_random(bounds)
        q_near = nearest(tree, q_rand)
        q_new = steer(q_near.q, q_rand)
        if collision_check(q_near.q, q_new):  # implement robot-specific check
            tree.append(Node(q_new, q_near))
            if distance(q_new, q_goal) < 0.3:
                # connect to goal
                if collision_check(q_new, q_goal):
                    return reconstruct_path(Node(q_goal, tree[-1]))
    return None
# collision_check and reconstruct_path omitted (robot-specific).