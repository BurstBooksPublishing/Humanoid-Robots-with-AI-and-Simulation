import math

# Input parameters (example hip actuator)
T = 25.0            # torque N*m (tightening spec)
d = 0.010           # nominal bolt diameter m
K = 0.2             # torque coefficient (lubricated)
A_t = math.pi*(d/2)**2  # tensile area m^2
A_s = 0.7854*d**2*0.6   # approximate shear area m^2 (threaded region)

# Compute preload from torque law
F_p = T / (K * d)   # N

# Load conditions
M_r = 12.0          # moment transmitted N*m
r_eff = 0.015       # effective radius m
mu = 0.15           # friction coefficient

# Safety checks
sigma_a = F_p / A_t          # axial stress Pa
tau = (M_r / r_eff) / A_s    # induced shear stress Pa
shear_margin = (0.6e9 - tau)/1e6  # margin to steel shear strength (MPa)

print("# preload (N):", F_p)               # monitor during assembly
print("# axial stress (MPa):", sigma_a/1e6)
print("# shear stress (MPa):", tau/1e6)
print("# shear margin (MPa):", shear_margin)
# Use sensor feedback to verify torque and record in assembly log.