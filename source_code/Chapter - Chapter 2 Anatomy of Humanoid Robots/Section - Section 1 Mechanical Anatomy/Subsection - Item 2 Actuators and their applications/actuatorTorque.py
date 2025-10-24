import numpy as np

# model placeholders (replace with real robot dynamics)
def gravity_torque(theta): 
    return 9.81 * 0.5 * np.cos(theta)  # simple pendulum approx.

def inertia(theta):
    return 0.05  # kg*m^2 constant inertia

# controller gains
Kp = 80.0  # Nm/rad
Kd = 2.0   # Nm*s/rad

# desired and measured states
theta_des = 0.1
theta = 0.12
theta_dot = -0.05
theta_ddot_des = 0.0

# PD + feedforward accel + gravity compensation
tau_pd = Kp*(theta_des - theta) + Kd*(0.0 - theta_dot)
tau_ff = inertia(theta) * theta_ddot_des
tau_grav = gravity_torque(theta)
tau_command = tau_pd + tau_ff + tau_grav

print(f"Torque command: {tau_command:.3f} Nm")  # send to low-level current controller