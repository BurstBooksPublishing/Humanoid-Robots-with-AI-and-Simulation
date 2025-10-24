import numpy as np
import time
# adjacency and initial poses (2D for brevity)
A = np.array([[0,1,1],
              [1,0,1],
              [1,1,0]])  # fully connected small team
poses = np.array([[0.0,0.0],[1.0,0.0],[0.5,0.8]])  # x,y per agent
eps = 0.1  # step size (must < 1/d_max)
def neighbor_states(i):
    return [poses[j] for j in range(len(poses)) if A[i,j]]
for k in range(100):
    new = poses.copy()
    for i in range(len(poses)):
        nbrs = neighbor_states(i)
        if not nbrs:
            continue
        mean_nbr = np.mean(nbrs, axis=0)
        new[i] = poses[i] + eps * (mean_nbr - poses[i])  # eqn (1)
    poses[:] = new
    time.sleep(0.01)  # simulate communication delay
# final poses converge toward centroid
print("Converged poses:", poses)