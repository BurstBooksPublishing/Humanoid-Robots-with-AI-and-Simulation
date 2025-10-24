import numpy as np

def construct_state(sim):
    # read sensors from simulator; sim.* are wrappers for hardware/sim APIs
    q = sim.get_joint_positions()         # vector of joint angles
    qd = sim.get_joint_velocities()       # joint velocities
    imu_ori = sim.get_base_orientation()  # quaternion
    imu_omega = sim.get_base_angular_velocity()
    com = sim.estimate_com()               # center of mass (3D)
    foot_contacts = sim.get_foot_contacts()# boolean per foot
    # derived features
    com_height = com[2]
    zmp_err = sim.estimate_zmp_error()     # scalar
    # concatenate normalized features
    state = np.concatenate([q, qd, imu_ori, imu_omega, [com_height, zmp_err], foot_contacts])
    return state

def compute_reward(state, action, next_state):
    # unpack features by known indices (kept short here)
    com_height = next_state[-(2+len(foot_contact))]  # example index use
    zmp_err = next_state[-(1+len(foot_contact))]
    foot_contact_sum = next_state[-len(foot_contact):].sum()
    # reward terms
    stability = -abs(zmp_err)                    # smaller is better
    height_pref = -abs(com_height - 1.0)         # prefer nominal height 1.0 m
    energy_penalty = -0.001 * np.sum(np.square(action))  # discourage high torques
    fall_penalty = -100.0 if foot_contact_sum == 0 else 0.0  # heavy penalty on fall
    # weighted sum
    reward = 2.0*stability + 1.0*height_pref + energy_penalty + fall_penalty
    return reward