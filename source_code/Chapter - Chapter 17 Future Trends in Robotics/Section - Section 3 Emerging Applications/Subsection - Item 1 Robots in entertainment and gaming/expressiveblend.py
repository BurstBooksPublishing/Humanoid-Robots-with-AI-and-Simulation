import time
# assume sensor_interface, controller, planner are provided interfaces
LOOP_HZ = 200
dt = 1.0 / LOOP_HZ
while True:
    t0 = time.time()
    state = sensor_interface.read_state()              # encoders, imu, vision
    audience = sensor_interface.estimate_audience()   # may have latency
    # predict audience pose to compensate latency
    pred_aud = audience.pose + audience.vel * audience.measured_latency
    # planner returns target joint angles and expression weight (0..1)
    q_d, expr_w = planner.get_target(pred_aud)        # gesture primitive selector
    # blend micro-expression overlay (small offsets) into q_d
    micro = planner.micro_expression(expr_w)
    q_blend = q_d * (1 - expr_w) + (q_d + micro) * expr_w
    # compute torque using PD + feedforward (controller handles limits)
    tau = controller.compute_torque(q_blend, state)
    controller.send_torque(tau)
    # simple watchdog and safety check
    if not sensor_interface.ok(): controller.emergency_stop()
    # enforce loop timing
    elapsed = time.time() - t0
    time.sleep(max(0, dt - elapsed))