# setup environment and policy (pseudo-code)
env = IsaacSimEnv(config)  # visual, tactile, proprioception
policy = initialize_policy()  # e.g., PPO with MLP or CNN encoder

for episode in range(num_episodes):
    obs = env.reset()  # includes image, joints, tactile
    done = False
    while not done:
        action = policy.sample(obs)  # stochastic during training
        next_obs, reward, done, info = env.step(action)  # physics steps
        policy.store_transition(obs, action, reward, next_obs, done)  # buffer
        obs = next_obs
    policy.update()  # batch update with advantage estimates
    if episode % eval_interval == 0:
        evaluate_policy(policy, env_eval)  # safety and success metrics