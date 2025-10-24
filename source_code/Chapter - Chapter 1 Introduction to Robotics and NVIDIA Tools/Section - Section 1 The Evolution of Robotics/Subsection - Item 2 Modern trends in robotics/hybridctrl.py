import numpy as np
# get robot state (q, qdot) from sensors // real-time update
q, qdot = read_joint_state()                # read sensors
# model-based terms from dynamics library (fast C++ backend) 
g = dynamics.gravity_vector(q)              # gravity term
M = dynamics.mass_matrix(q)                 # mass matrix (if needed)
# PD setpoint
qd = desired_position()
qd_dot = np.zeros_like(q)
Kp = np.diag([200.0]*len(q))                # stiffness gains
Kd = np.diag([10.0]*len(q))                 # damping gains
tau_pd = Kp.dot(qd - q) + Kd.dot(qd_dot - qdot)  # PD feedback
# learned residual policy (inference on GPU/edge)
state = np.concatenate([q, qdot, sensor_obs()])
tau_learn = policy_model.infer(state)       # learned correction
alpha = 0.6                                 # blending factor (safety-tuned)
# final torque command with gravity compensation
tau_cmd = tau_pd + g + alpha * tau_learn
send_torque_command(tau_cmd)                # low-level actuator interface