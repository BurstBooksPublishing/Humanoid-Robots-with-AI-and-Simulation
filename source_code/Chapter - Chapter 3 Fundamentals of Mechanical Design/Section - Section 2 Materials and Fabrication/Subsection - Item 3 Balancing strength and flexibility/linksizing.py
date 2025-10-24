import numpy as np

# Inputs (replace with robot-specific values)
M = 50.0           # bending moment [N·m]
L = 0.30           # length [m]
b = 0.02           # width [m]
E = 70e9           # Young's modulus [Pa] (aluminum)
rho = 2700         # density [kg/m^3]
sigma_allow = 250e6 # allowable stress [Pa]
K = 1.0            # Euler effective length factor

# solve for thickness t to meet sigma_allow: M*c/I <= sigma_allow
# for rectangle, I = b*t^3/12 and c = t/2 => sigma = 6*M/(b*t^2)
t_req = np.sqrt(6*M/(b*sigma_allow))  # [m]
# Euler buckling check for axial compressive load P (example)
P = 200.0          # compressive load [N]
I = b*t_req**3/12.0
Pcr = np.pi**2 * E * I / (K*L)**2

print(f"required thickness t = {t_req*1000:.2f} mm")   # thickness in mm
print(f"Euler buckling Pcr = {Pcr:.1f} N, margin = {Pcr/P:.1f}")  # safety margin