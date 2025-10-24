import numpy as np

# example inputs: masses (kg) and per-link COM positions in base frame (m)
masses = np.array([12.0, 3.5, 2.0, 2.0, 4.0, 1.2, 1.2])  # trunk, left/right thigh, shank, upperarms, forearms...
com_positions = np.array([
    [0.0, 0.0, 0.45],   # trunk COM
    [0.0, 0.12, 0.0],   # left thigh
    [0.0, -0.12, 0.0],  # right thigh
    [0.0, 0.12, -0.4],  # left shank
    [0.0, -0.12, -0.4], # right shank
    [0.2, 0.15, 0.1],   # left arm
    [0.2, -0.15, 0.1],  # right arm
])
M = masses.sum()
# weighted sum of COMs
global_com = (masses[:,None] * com_positions).sum(axis=0) / M
print("Total mass:", M)                # quick check
print("Global COM (m):", global_com)   # use in balance planner