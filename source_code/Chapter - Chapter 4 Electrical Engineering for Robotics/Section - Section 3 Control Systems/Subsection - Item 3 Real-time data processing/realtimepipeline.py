import time, threading, collections, math
# ring buffer for latest IMU samples
imu_buf = collections.deque(maxlen=256)

def imu_reader():                        # sensor acquisition thread
    while True:
        sample = read_imu_hardware()     # blocking HAL call (stub)
        imu_buf.append((time.time(), sample))
        time.sleep(0.001)               # simulate 1 kHz IMU

def control_loop():                      # prototyping control thread
    Ts = 0.002                           # 500 Hz loop
    alpha = 0.98                         # complementary filter gain
    theta = 0.0
    next_time = time.time()
    while True:
        # real systems should poll latest sample atomically
        if imu_buf:
            tstamp, imu = imu_buf[-1]   # latest non-blocking read
            gyro_z = imu['gyro_z']
            acc_angle = math.atan2(imu['acc_y'], imu['acc_z'])
            # complementary filter (state estimation)
            theta = alpha*(theta + gyro_z*Ts) + (1-alpha)*acc_angle
            # simple PD control for torso pitch (example)
            desired = 0.0
            error = desired - theta
            u = Kp*error - Kd*(gyro_z)  # torque command
            send_torque_command(u)     # non-blocking HAL call
        next_time += Ts
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_time = time.time()    # missed deadline, resync