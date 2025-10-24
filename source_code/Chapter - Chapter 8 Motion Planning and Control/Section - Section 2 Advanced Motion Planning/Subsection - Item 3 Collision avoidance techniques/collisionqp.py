import cvxpy as cp
import numpy as np

# Inputs from perception & kinematics (examples)
J_task = np.array(...)   # task Jacobian (m x n)
v_task = np.array(...)   # desired task velocity (m,)
Jd_list = [np.array(...), ...]  # distance Jacobians for obstacles (1 x n)
d_list = [0.02, 0.5, ...]       # signed distances (m)
d_safe = 0.05                   # safety margin (m)
q = np.array(...)               # current joint angles (n,)
q_min = np.array(...)           # joint minima
q_max = np.array(...)           # joint maxima

n = q.size
# Decision variable: joint increment
dq = cp.Variable(n)

# Objective: track task velocity, regularize joint change
lambda_reg = 1e-3
obj = cp.Minimize(cp.sum_squares(J_task @ dq - v_task) + lambda_reg * cp.sum_squares(dq))

# Constraints: linearized collision and joint limits
constraints = []
for Jd, d in zip(Jd_list, d_list):
    # Jd @ dq >= d_safe - d  ->  -Jd @ dq <= d - d_safe
    constraints.append(-Jd @ dq <= d - d_safe)

constraints += [q + dq >= q_min, q + dq <= q_max]

# Solve
prob = cp.Problem(obj, constraints)
prob.solve(solver=cp.OSQP, warm_start=True)

dq_cmd = dq.value  # joint update to apply (with smoothing)