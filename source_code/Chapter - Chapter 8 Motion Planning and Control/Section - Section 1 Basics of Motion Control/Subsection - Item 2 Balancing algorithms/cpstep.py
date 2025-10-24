import numpy as np

# parameters
g = 9.81
z_c = 0.9
omega = np.sqrt(g / z_c)
step_time = 0.6  # seconds
max_step_reach = 0.4  # meters

def capture_point(x, xdot):
    return x + xdot / omega

def propose_step(x, xdot, p_current):
    xi = capture_point(x, xdot)
    # analytic one-step target using simple exponential model
    x_step = xi + np.exp(-omega * step_time) * (xi - p_current)
    # saturate to kinematic reach
    dx = np.clip(x_step - p_current, -max_step_reach, max_step_reach)
    return p_current + dx

# Example usage: current CoM pos/vel and current ZMP
x = np.array([0.0, 0.0])      # CoM horizontal (x,y)
xdot = np.array([0.3, 0.0])   # CoM velocity
p_cur = np.array([0.0, 0.0])  # current ZMP
step_target = propose_step(x[0], xdot[0], p_cur[0])
print("Proposed step x:", step_target)