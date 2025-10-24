import numpy as np

# inputs: J (6xn), M (nxn), q_dot (n,), f_des (6,), tau_posture_des (n,)
# simple whole-body torque computation
J = np.array(...)           # end-effector jacobian
M = np.array(...)           # mass matrix
f_des = np.array(...)       # desired spatial force
tau_posture = np.array(...) # nullspace posture torque

# compute torque to realize end-effector force
tau_task = J.T @ f_des      # mapped task torque

# nullspace projector (damped pseudoinverse)
lambda_damp = 1e-4
Jpinv = np.linalg.solve(J @ J.T + lambda_damp*np.eye(6), J)  # 6x n
N = np.eye(J.shape[1]) - Jpinv.T @ J                       # nullspace projector

# final torque command: task torque + nullspace posture torque
tau_cmd = tau_task + N @ tau_posture

# apply actuator saturation
tau_cmd = np.clip(tau_cmd, -np.ones_like(tau_cmd)*100, 100) # clip torques