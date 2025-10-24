# main loop (pseudo-ROS2 node)
while True:
    obs = get_obstacle_estimates()           # fast sensor fusion
    x_hat = get_state_estimate()             # robot pose & velocity
    t_c = estimate_ttc(x_hat, obs)           # eq. (3) approximate
    sigma = prediction_uncertainty(obs)      # covariance metric

    # compute reactive action quickly (eq. 1)
    u_react = reactive_controller(x_hat, obs)    # safety reflex

    # dynamic allocation of compute for MPC
    if t_c > T_proactive and sigma < sigma_th:
        # run MPC asynchronously; warm-start with previous solution
        u_mpc = solve_mpc_async(x_hat, obs, horizon=N)  # eq. (2)
    else:
        u_mpc = None

    # blending strategy (eq. 4)
    alpha = compute_alpha(t_c, sigma)        # in [0,1]
    if u_mpc is not None:
        u = alpha*u_react + (1-alpha)*u_mpc
    else:
        u = u_react                           # fallback reflex

    send_actuator_commands(u)                # low-level execution