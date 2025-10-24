import numpy as np

# example parameters (SI units)
m = 2.5            # link mass (kg)
r = 0.25           # COM distance from joint (m)
L = 0.5            # link length (m)
theta_ddot = 10.0  # peak angular accel (rad/s^2)
g = 9.81

# compute inertias and torques
I = (1/3.0) * m * L**2                     # slender rod approx
tau_g = m * g * r                          # eq. (1)
tau_dyn = (I + m * r**2) * theta_ddot      # acceleration term
tau_req = tau_dyn + tau_g                  # eq. (2)

# actuator candidate
motor_nominal_torque = 1.2                 # Nm (continuous)
gear_ratio = 50.0
motor_peak_torque = 3.6                    # Nm (peak)
tau_selected = motor_peak_torque * gear_ratio

# margins and natural frequency (assume stiffness)
margin = (tau_selected - tau_req) / tau_req
k_joint = 500.0                             # Nm/rad (effective stiffness)
J_eff = I + m * r**2                        # effective inertia seen by joint
omega_n = np.sqrt(k_joint / J_eff)

# print results (for integration with design doc)
print(f"tau_req={tau_req:.2f} Nm, tau_selected={tau_selected:.1f} Nm, margin={margin:.2f}")
print(f"natural_freq={omega_n:.2f} rad/s")