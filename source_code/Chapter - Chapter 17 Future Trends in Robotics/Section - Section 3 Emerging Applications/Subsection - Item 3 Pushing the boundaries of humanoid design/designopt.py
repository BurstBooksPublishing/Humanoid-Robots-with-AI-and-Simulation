import numpy as np
from scipy.optimize import minimize

# parameters (realistic engineering values)
P_avg = 150.0         # W average power draw (locomotion+compute)
T_mission = 2*3600.0  # seconds (2 hours)
e_batt = 180.0        # Wh/kg -> convert to J/kg below
e_batt_J = e_batt * 3600.0

# structural mass scales with battery mass (empirical)
def total_mass(m_batt):
    m_struct = 20.0 + 0.3 * m_batt   # kg, base plus proportional growth
    m_payload = 5.0
    return m_batt + m_struct + m_payload

# objective: minimize total mass
def obj(x):
    m_batt = x[0]
    return total_mass(m_batt)

# constraint: stored energy must meet mission energy
def energy_constraint(x):
    m_batt = x[0]
    E_avail = m_batt * e_batt_J
    E_req = P_avg * T_mission
    return E_avail - E_req  # must be >= 0

# bounds and initial guess
x0 = [10.0]  # initial battery mass guess
bounds = [(1.0, 200.0)]
cons = ({'type': 'ineq', 'fun': energy_constraint})

res = minimize(obj, x0, bounds=bounds, constraints=cons)
m_batt_opt = res.x[0]
print("Optimal battery mass (kg):", m_batt_opt)  # simple design metric