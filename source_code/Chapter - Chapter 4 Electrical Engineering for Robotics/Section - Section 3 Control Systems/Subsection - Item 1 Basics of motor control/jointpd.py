import time
# Gains (tuned per joint)
Kp = 150.0
Kd = 5.0
max_tau = 25.0    # Nm, torque limit
sample_dt = 0.002 # 2 ms loop

def gravity_comp(theta): 
    # approximate gravity torque for link; replace with model
    return 9.81 * 0.5 * 1.0 * np.sin(theta)  # m*g*com*mass*sin

prev_error = 0.0
prev_theta = 0.0

while True:
    t0 = time.time()
    theta, theta_dot = read_sensors()   # encoder + differentiator/IMU fusion
    theta_des, theta_dot_des, tau_ff = compute_trajectory()  # planner
    
    # PD control law with feedforward
    error = theta_des - theta
    derror = (error - prev_error) / sample_dt
    tau_cmd = Kp * error + Kd * (theta_dot_des - theta_dot) + tau_ff
    # gravity compensation added if needed
    tau_cmd += gravity_comp(theta)
    
    # safety clamps and anti-windup
    tau_cmd = max(min(tau_cmd, max_tau), -max_tau)
    
    send_torque(tau_cmd)  # drive API; inner current loop enforces torque
    prev_error = error
    # enforce precise loop timing
    elapsed = time.time() - t0
    time.sleep(max(0.0, sample_dt - elapsed))