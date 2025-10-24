for episode in range(num_episodes):
    # sample dynamics params (center ± scale)
    params = {
        "mass_scale": np.random.uniform(0.9, 1.2),
        "friction": np.random.uniform(0.4, 1.2),
        "joint_damping": np.random.uniform(0.7, 1.3),
        "action_delay": np.random.randint(0, 3),
        "imu_noise_std": np.random.uniform(0.01, 0.08),
    }
    env.apply_physics_params(params)  # map to Isaac Sim scene / actor properties
    obs = env.reset()
    done = False
    while not done:
        action = policy.act(obs)               # neural policy inference
        action = apply_action_delay(action)    # emulate actuator latency
        obs, reward, done, info = env.step(action)  # simulator step
        replay_buffer.add(obs, action, reward)
    trainer.update(replay_buffer)              # RL algorithm update