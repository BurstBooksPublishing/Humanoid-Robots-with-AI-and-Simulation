# read target and encoder, compute PID, send torque command
from time import sleep, perf_counter

Kp, Ki, Kd = 50.0, 10.0, 0.5  # tuned for joint dynamics
dt = 0.005                      # 200 Hz outer loop
integrator = 0.0
last_error = 0.0
last_time = perf_counter()

while robot_running():
    t0 = perf_counter()
    # read sensors (blocking or DMA), small latency required
    q = read_encoder()           # joint angle
    qdot = read_velocity()       # optional velocity estimate
    q_ref = get_reference()      # trajectory point

    e = q_ref - q
    integrator += e * dt
    derivative = (e - last_error) / dt

    tau_ff = compute_inverse_dynamics(q_ref, qdot)  # feedforward torque
    tau_cmd = Kp*e + Ki*integrator + Kd*derivative + tau_ff

    # safety: torque and current limits
    tau_cmd = max(min(tau_cmd, MAX_TORQUE), -MAX_TORQUE)

    send_torque_command(tau_cmd)  # to inner current controller

    last_error = e
    # maintain loop timing
    sleep(max(0.0, dt - (perf_counter()-t0)))