import numpy as np
import osqp
from scipy import sparse

# u_nom: nominal velocity vector from planner (2D linear, 1D angular)
# x: current state; obst: list of obstacles with mean+cov
def solve_safety_qp(u_nom, x, obst):
    # cost: minimize deviation from u_nom
    P = sparse.csc_matrix(2.0 * np.eye(len(u_nom)))
    q = -2.0 * u_nom  # linear term for ||u-u_nom||^2

    # build linear constraints A u <= b (example: velocity bounds)
    A_list, b_list = [], []
    # velocity bounds
    A_list.append(np.vstack([np.eye(len(u_nom)), -np.eye(len(u_nom))]))
    b_list.append(np.hstack([np.array([0.8, 1.0, 0.5]), -np.array([-0.8, -1.0, -0.5])]))
    # collision constraints: linearized distance constraint n^T u <= beta
    for o in obst:
        n, beta = linearize_collision_constraint(x, o)  # implement separately
        A_list.append(n.reshape(1, -1))
        b_list.append(np.array([beta]))
    A = sparse.csc_matrix(np.vstack(A_list))
    b = np.hstack(b_list)

    # setup OSQP
    prob = osqp.OSQP()
    prob.setup(P, q, A, b, verbose=False)
    res = prob.solve()
    if res.info.status != 'solved':
        return u_nom  # fallback to nominal if QP fails
    return res.x  # filtered command