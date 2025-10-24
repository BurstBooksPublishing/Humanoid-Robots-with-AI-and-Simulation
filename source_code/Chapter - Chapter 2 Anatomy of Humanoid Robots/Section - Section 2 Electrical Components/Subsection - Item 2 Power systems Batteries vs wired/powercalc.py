# Compute battery and cable sizing for humanoid robot mission (illustrative).
import math

# Inputs (engineer-assigned)
P_avg = 300.0       # average power [W]
t_hours = 1.0       # mission time [h]
V_nom = 48.0        # nominal bus voltage [V]
e_spec = 200.0      # battery specific energy [Wh/kg]
rho_cu = 1.68e-8    # copper resistivity [Ohm*m]
L_single = 2.0      # single conductor length [m]
I_peak = 30.0       # peak current [A]
alpha = 0.02        # allowable voltage drop fraction

# Calculations
E_req = P_avg * t_hours              # Wh
Q_Ah = E_req / V_nom                 # Ah
m_batt = E_req / e_spec              # kg
A_min = (I_peak * rho_cu * L_single) / (alpha * V_nom)  # m^2

# Outputs (engineer interprets)
print(f"Required energy: {E_req:.1f} Wh")
print(f"Battery capacity: {Q_Ah:.2f} Ah at {V_nom} V")
print(f"Estimated battery mass: {m_batt:.2f} kg")
print(f"Minimum conductor area: {A_min*1e6:.2f} mm^2")  # convert to mm^2
# Further: map A_min to standard gauge, evaluate thermal run rate, choose local buffering