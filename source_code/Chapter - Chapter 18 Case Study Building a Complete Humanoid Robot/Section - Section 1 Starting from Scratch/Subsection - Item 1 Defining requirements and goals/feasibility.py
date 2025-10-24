import numpy as np

# inputs (example scenario)
P_avg = 250.0              # W average power draw
t_mission = 3600.0         # s mission duration (1 hour)
eta_sys = 0.85             # system efficiency
e_density = 200.0          # Wh/kg usable energy density

# battery compute (Wh -> convert to Wh by P_avg*hours)
E_req_Wh = P_avg * (t_mission/3600.0) / eta_sys
battery_mass = E_req_Wh / e_density  # kg

# torque margin check for single joint
I = 0.02                   # kg*m^2 inertia of link
alpha = 2.0                # rad/s^2 required accel
m_link = 3.0               # kg
l_cm = 0.12                # m
theta = 0.0                # rad (worst-case)
g = 9.81
tau_req = I*alpha + m_link*g*l_cm*np.cos(theta)
safety_factor = 2.0
motor_tau = 50.0           # Nm available at gearbox

# print checks (inline comments explain outputs)
print(f"E_req_Wh={E_req_Wh:.1f} Wh, battery_mass={battery_mass:.2f} kg")  # energy check
print(f"tau_req={tau_req:.2f} Nm, required_with_sf={tau_req*safety_factor:.2f} Nm")  # torque check
print("torque_OK=", motor_tau >= tau_req*safety_factor)  # boolean feasibility