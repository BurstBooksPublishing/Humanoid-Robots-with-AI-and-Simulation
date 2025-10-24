import numpy as np
# qp solver and sensor interfaces are assumed installed
import cvxpy as cp

# setup dims, horizon, cost matrices
N = 10
nx = 12  # example whole-body reduced state (CoM, base, key joints)
nu = 6
Q = np.eye(nx) * 10.0
R = np.eye(nu) * 0.1

def linearize_dynamics(x0):
    A = np.eye(nx)  # placeholder linearization
    B = np.zeros((nx,nu))
    return A, B

def predict_obstacles(obs_list, dt, N):
    # constant-velocity prediction, returns list of positions per horizon step
    preds = []
    for k in range(N):
        preds.append([o['pos'] + (k+1)*dt*o['vel'] for o in obs_list])
    return preds

def build_qp(x0, x_ref, obs_preds, dt):
    # decision vars
    U = cp.Variable((nu, N))
    X = cp.Variable((nx, N+1))
    cost = 0
    cons = []
    cons += [X[:,0] == x0]
    A, B = linearize_dynamics(x0)
    for k in range(N):
        cost += cp.quad_form(X[:,k] - x_ref[:,k], Q) + cp.quad_form(U[:,k], R)
        cons += [X[:,k+1] == A @ X[:,k] + B @ U[:,k]]
        # obstacle linear constraints (conservative sphere around key link)
        for obs in obs_preds[k]:
            # s: signed-distance gradient approx towards link; r=robot_radius
            s = np.array([1.0] + [0]*(nx-1))  # simplistic example toward CoM
            r = 0.3
            cons += [s @ X[:,k] + (-obs[0]) >= r]  # linearized avoid constraint
        # actuator limits
        cons += [U[:,k] >= -1.0, U[:,k] <= 1.0]
    prob = cp.Problem(cp.Minimize(cost), cons)
    return prob, U

# main loop (pseudo)
# subscribe to sensors -> obs_list; get x0 and x_ref
# obs_preds = predict_obstacles(obs_list, dt=0.05, N=N)
# prob, U = build_qp(x0, x_ref, obs_preds, dt=0.05)
# prob.solve(solver=cp.OSQP, warm_start=True)
# send first column of U as control command