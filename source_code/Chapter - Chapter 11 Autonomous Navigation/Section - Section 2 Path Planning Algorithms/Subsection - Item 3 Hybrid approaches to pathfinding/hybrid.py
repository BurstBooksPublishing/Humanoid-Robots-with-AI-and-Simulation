# Hybrid planner loop: global A*, local RRT* + optimizer, MPC tracking
while not at_goal(): 
    global_path = global_planner.compute(start, goal)           # coarse A*
    local_window = extract_window(global_path, robot_pose)      # sensor horizon
    # sampling-based local planner (kinodynamic)
    sampled_traj = local_sampler.rrt_star(local_window, robot_state) 
    if sampled_traj is None:                                     # fallback replan
        control_reference = follow_global_coarse(global_path)    # safe fallback
    else:
        # trajectory optimizer refines sampled trajectory to satisfy dynamics
        opt_traj = trajectory_optimizer.solve(sampled_traj, constraints) 
        control_reference = mpc_controller.track(opt_traj)      # short-horizon MPC
    executer.send_reference(control_reference)                  # actuators
    robot_state = state_estimator.update()                       # sensors + filtering
    if significant_change_detected():                           # dynamic obstacles
        start = robot_pose                                        # replan immediately