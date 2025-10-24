import numpy as np

def compute_reward(obs, action, next_obs, running_stats, params):
    # obs contains sensors: imu_ori, com_vel, foot_contacts, joint_torques, contact_impulses
    # running_stats holds running mean/std for normalization
    # params contains weights and target velocity

    # Uprightness term (dot of torso z and world z)
    upright = max(0.0, np.dot(obs['torso_z'], np.array([0.,0.,1.])))  # [0,1]

    # COM velocity tracking (quadratic penalty)
    v_err = next_obs['com_vel'] - params['v_target']
    com_term = -params['alpha'] * np.dot(v_err, v_err)

    # Energy penalty (squared torques)
    energy_term = -params['beta'] * np.sum(np.square(action))  # torque cost

    # Contact safety (penalize large impulses)
    contact_term = -params['gamma'] * np.sum(np.square(next_obs['contact_impulses']))

    # Footstep reward (binary reward for correct contact pattern)
    foot_reward = params['foot_bonus'] * (next_obs['correct_foot_phase'] * 1.0)

    # Potential-based shaping: progress along forward axis
    phi_s = np.dot(obs['com_pos'], params['forward_axis'])
    phi_s1 = np.dot(next_obs['com_pos'], params['forward_axis'])
    shaping = params['discount'] * phi_s1 - phi_s

    # Normalize components using running statistics
    upright_n = (upright - running_stats['upright_mean']) / (running_stats['upright_std'] + 1e-6)
    com_n = com_term / (running_stats['com_scale'] + 1e-6)

    # Weighted sum
    reward = (params['w_upright'] * upright_n +
              params['w_com'] * com_n +
              params['w_energy'] * energy_term +
              params['w_contact'] * contact_term +
              params['w_foot'] * foot_reward +
              params['w_shaping'] * shaping)

    # Survival/fall handling
    if next_obs['fell']:
        reward -= params['fall_penalty']

    # Clip reward to avoid exploding returns
    return float(np.clip(reward, -params['reward_clip'], params['reward_clip']))