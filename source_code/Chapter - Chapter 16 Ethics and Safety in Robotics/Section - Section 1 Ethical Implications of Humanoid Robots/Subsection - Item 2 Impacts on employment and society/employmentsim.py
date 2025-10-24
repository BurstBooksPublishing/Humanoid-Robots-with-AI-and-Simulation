import numpy as np
import matplotlib.pyplot as plt

# Parameters (t in years)
years = np.linspace(0,20,201)
E0 = 10000                 # initial employed humans
A_max = 0.4                # max automated fraction (Eq.1 estimate)
k = 0.5                    # adoption steepness
t0 = 5.0                   # adoption midpoint
gamma = 0.3                # displacement elasticity
eta = 0.7                  # conversion efficiency of growth to jobs
G = 50                     # constant new jobs per year (macro effect)

def adoption(t):           # logistic adoption curve r(t)
    return 1.0/(1.0+np.exp(-k*(t-t0)))

E = np.zeros_like(years)
E[0] = E0
dt = years[1]-years[0]
for i in range(1, len(years)):
    r = adoption(years[i])
    A_t = r * A_max                        # time-varying automation exposure
    dE = (-gamma * A_t * E[i-1] + eta * G) * dt
    E[i] = max(0, E[i-1] + dE)             # employment cannot be negative

# Plotting (for reports)
plt.plot(years, E); plt.xlabel('Years'); plt.ylabel('Employed workers')
plt.title('Employment trajectory under humanoid adoption'); plt.grid(True)
plt.show()