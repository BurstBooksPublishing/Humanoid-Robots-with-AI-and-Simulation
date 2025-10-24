# pseudocode: run on each humanoid
while True:
    state = sense_state()                         # local sensors, IMU, kinematics
    neighbors = recv_neighbors()                  # receive neighbor states
    # consensus update for shared variable x (e.g., meeting point)
    x = state.x
    for nb in neighbors:
        x += alpha * w(state.id, nb.id) * (nb.x - x)  # Eq. (1)
    state.x = x

    # compute formation force on ground-plane (p: x,y)
    force = np.zeros(2)
    for nb in neighbors:
        r = state.p - nb.p
        d = np.linalg.norm(r) + 1e-6
        force += k_r * r / (d**3)                   # repulsive gradient
    force += -k_a * (state.p - state.p_ref)         # attraction term (Eq. 2)

    # project desired planar force into feasible body motion
    desired_velocity = project_to_feasible_motion(force)  # respects balance
    # safety supervisor enforces ZMP and joint limits
    if safety_veto(desired_velocity):                 # veto if unstable
        desired_velocity = safe_stop_velocity()

    send_control(desired_velocity)                    # low-level controller
    publish_state(state)                              # neighbor updates
    sleep(control_dt)                                 # real-time loop