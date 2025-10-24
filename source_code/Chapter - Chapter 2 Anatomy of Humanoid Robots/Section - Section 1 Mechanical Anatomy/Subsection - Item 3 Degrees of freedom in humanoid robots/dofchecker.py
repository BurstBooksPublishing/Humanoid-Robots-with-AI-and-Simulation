import numpy as np

# Define DoF per group (example realistic humanoid)
dof = {'head':3, 'torso':3, 'arm':7, 'leg':6, 'hand':20}
# Aggregate actuated DoF
N_act = dof['head'] + dof['torso'] + 2*dof['arm'] + 2*dof['leg'] + dof['hand']
# Task specification: e.g., right-arm pose (6), COM control (3)
task_dims = {'right_arm_pose':6, 'com_position':3}
m_task = sum(task_dims.values())

# Floating base contribution for whole-body dynamics
N_base = 6  # virtual DoF for free-floating base
J_rows = m_task
J_cols = N_act  # Jacobian for actuated joints (base handled separately in dynamics)

# Sanity checks
print(f"Total actuated DoF: {N_act}")                 # sum check
print(f"Task dimension m: {m_task}")                  # task size
print(f"Redundancy margin r = N_act - m: {N_act-m_task}")  # redundancy

# Jacobian shape expected in kinematic control
J_shape = (J_rows, J_cols)
print("Expected Jacobian shape:", J_shape)

# Example minimal rank test (random Jacobian sample)
J_sample = np.random.randn(*J_shape)
rank = np.linalg.matrix_rank(J_sample)
print("Sample Jacobian rank:", rank, "-> feasible if rank==m")