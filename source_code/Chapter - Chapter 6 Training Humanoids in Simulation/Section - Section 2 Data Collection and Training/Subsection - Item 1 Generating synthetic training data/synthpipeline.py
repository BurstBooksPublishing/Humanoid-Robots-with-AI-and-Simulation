# create sim, load scene and humanoid model
sim = create_isaac_sim()                # init simulator
human = sim.spawn_humanoid('atl_model')# spawn humanoid virtual twin

for seed in range(num_seeds):           # deterministic randomness per seed
    params = sample_scenario(seed)      # sample env, lighting, gait, disturbances
    sim.apply_randomization(params)     # apply variations to scene and sensors
    sim.reset(seed=seed)                # reset physics state

    for t in range(steps_per_seed):     # step and record streams
        action = policy_or_script(t, params)   # scripted or random actions
        sim.step(action)               # physics step
        # capture multimodal data (RGB, depth, seg, IMU, joints, contacts)
        rgb = sim.capture_rgb()        # raw image
        depth = sim.capture_depth()    # depth map
        seg = sim.capture_segmentation()# per-pixel labels
        imu = sim.read_imu()           # noisy IMU model
        joints = sim.read_joint_states()
        contacts = sim.read_contact_forces()
        # write synchronized record with metadata and seed
        save_record(seed, t, rgb, depth, seg, imu, joints, contacts)
# finalize and close
sim.shutdown()