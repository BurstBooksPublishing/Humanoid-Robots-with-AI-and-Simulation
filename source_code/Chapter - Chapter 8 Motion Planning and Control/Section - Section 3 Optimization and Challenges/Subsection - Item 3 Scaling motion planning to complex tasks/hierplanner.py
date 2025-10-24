# high-level task sequence (symbolic) -> list of subgoals
task_sequence = plan_task(symbolic_world, goal)  # e.g., ['walk','reach','grasp']

for subgoal in task_sequence:
    # sample feasible mode and initial geometric path (PRM/RRT)
    mode = sample_contact_mode(subgoal)            # foot/hand contacts
    init_path = sampling_planner(robot_model, mode) # collision-free waypoint path

    # form optimization variables for local horizon
    X0, U0 = generate_initial_guess(init_path)     # warm start

    # run sequential convex optimization (SCP) with contact penalties
    Xopt, Uopt = trajectory_optimize(X0, U0,
                                     dynamics=model_dynamics, 
                                     constraints=contact_and_collision,
                                     warm_start=True)        # returns feasible local plan

    if Xopt is None:
        # fallback: try alternative modes or expand sampling
        continue

    execute_or_store(Xopt, Uopt)                   # send to controller or cache