import cvxpy as cp
import numpy as np

# inputs from model/estimator (linearized)
A = np.eye(6)          # state matrix (placeholder)
B = np.zeros((6,12))   # input matrix mapping torques to centroidal accel
x = np.zeros(6)        # current reduced state
x_ref = np.zeros(6)    # desired next state
tau_min = -50*np.ones(12)  # torque limits (Nm)
tau_max =  50*np.ones(12)
R = 1e-3*np.eye(12)    # torque cost weight
Q = 1.0*np.eye(6)      # state error weight

# decision variable
tau = cp.Variable(12)  # 12 actuators in reduced model

# dynamics constraint (single-step; linearized)
x_next = A @ x + B @ tau
cost = cp.quad_form(tau, R) + cp.quad_form(x_next - x_ref, Q)

constraints = [tau >= tau_min, tau <= tau_max]

prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve(solver=cp.OSQP, warm_start=True)  # warm start for speed

# tau.value gives the torque command to send to low-level controller