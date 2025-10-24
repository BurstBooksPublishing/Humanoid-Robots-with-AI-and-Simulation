import numpy as np
# env.step returns obs, reward, done, info
for episode in range(num_episodes):
    obs = env.reset()                             # reset sim state
    v_target = sample_velocity()                  # curriculum-sampled speed
    policy.set_command(v_target)                  # condition policy on speed
    done = False
    while not done:
        action = policy.act(obs)                  # forward pass
        next_obs, r, done, info = env.step(action) # sim step with contact model
        buffer.store(obs, action, r, done)        # store transition
        obs = next_obs
    # After rollout, compute advantages and update PPO policy
    advantages = compute_gae(buffer)               # GAE for on-policy updates
    policy.update(buffer.obs, buffer.actions, advantages) # gradient step
    buffer.clear()