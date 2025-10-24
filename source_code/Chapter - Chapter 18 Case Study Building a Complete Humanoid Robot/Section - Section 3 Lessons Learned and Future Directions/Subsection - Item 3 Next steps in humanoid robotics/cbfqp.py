import cvxpy as cp
import numpy as np

# simple planar dynamics linearization: x_dot = A x + B u
A = np.array([[0,1],[0,0]])
B = np.array([[0],[1]])
# nominal control (from MPC or policy)
u_nom = np.array([0.5])                # // nominal torque
x = np.array([0.2, -0.1])              # // current state

# barrier function h(x) = x1 - x_safe_min (must remain >=0)
h = x[0] - 0.0
# Lie derivatives approximated via linear dynamics
Lfh = A[0,:].dot(x)
Lgh = B[0,:].dot(np.ones(1))           # // simplified

u = cp.Variable((1,))
# class-K linear function alpha(h) = k*h
k = 5.0
constraints = [Lfh + Lgh*u + k*h >= 0] # // enforce CBF condition
# QP: minimize deviation from nominal control
prob = cp.Problem(cp.Minimize(cp.sum_squares(u - u_nom)),
                  constraints + [cp.abs(u) <= 2.0]) # // actuation limit

prob.solve(solver=cp.OSQP)
u_safe = u.value
# // apply u_safe to low-level controller