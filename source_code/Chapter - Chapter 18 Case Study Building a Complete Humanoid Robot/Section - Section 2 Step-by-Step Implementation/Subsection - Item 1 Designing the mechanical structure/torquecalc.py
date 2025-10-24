import numpy as np

# link params (from CAD): masses (kg), com distances (m), inertias (kg*m^2)
m1, m2 = 2.5, 1.8
l1, l2 = 0.24, 0.18
I1, I2 = 0.012, 0.006

# desired motion: peak accel (rad/s^2), worst-case posture angle (rad)
alpha = 6.0  # peak angular accel at shoulder
theta = 0.0  # horizontal arm (worst gravity torque)

g = 9.81
# equivalent inertia at shoulder (parallel axis + reflected distal inertia)
I_eq = I1 + m1*(l1**2) + I2 + m2*((l1+l2/2)**2)
# gravity term using distal masses (approx)
tau_grav = m1*g*l1*np.cos(theta) + m2*g*(l1+l2/2)*np.cos(theta)
tau_dyn = I_eq * alpha
tau_req = tau_dyn + tau_grav

# motor selection: motor continuous torque and required gearbox ratio
motor_tau_cont = 0.8  # Nm
safety_factor = 2.0
gear_ratio = (safety_factor * tau_req) / motor_tau_cont

print(f"Required torque: {tau_req:.2f} Nm")
print(f"Suggested gearbox ratio: {gear_ratio:.1f} : 1")
# Next steps: consult gearbox catalog and verify reflected inertia J_m*gear_ratio^2