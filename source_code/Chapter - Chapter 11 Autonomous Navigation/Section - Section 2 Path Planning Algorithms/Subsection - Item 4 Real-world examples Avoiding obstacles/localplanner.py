def plan_to_goal(start, goal, costmap, dyn_preds):
    # compute coarse global corridor with A* on costmap
    corridor = astar_global(start, goal, costmap)
    # sample-based local planning inside corridor
    tree = RRTStar(state_dim=4)                    # x,y,theta,vel
    tree.add_root(start)
    timeout = 0.5  # seconds
    while not tree.has_solution() and time_left(timeout):
        x_rand = sample_in_corridor(corridor)     # biased sampling
        x_near = tree.nearest(x_rand)
        x_new = steer_kinodynamic(x_near, x_rand) # respects accel limits
        if not collision_check_kin(x_new, costmap): continue
        if time_collides_with_preds(x_new, dyn_preds): continue
        tree.add_node(x_new, parent=x_near)
    traj = tree.reconstruct_path(goal)
    # convert continuous torso path to footsteps
    footsteps = footstep_planner(traj)            # discrete foot pair list
    # validate footsteps for kinematic reachability and balance
    for fs in footsteps:
        if not ik_solve(fs):                      # whole-body IK check
            # try local adjustment; fallback to replan if fails
            fs_adj = adjust_footstep(fs)
            if not ik_solve(fs_adj):
                return None                        # replan globally
        if not stability_margin_ok(fs):            # ensure ZMP margin
            return None                            # replan or slow down
    return assemble_motion_plan(traj, footsteps)