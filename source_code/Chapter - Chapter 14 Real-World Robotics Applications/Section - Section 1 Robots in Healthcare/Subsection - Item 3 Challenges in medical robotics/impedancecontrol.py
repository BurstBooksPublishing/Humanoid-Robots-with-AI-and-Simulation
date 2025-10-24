# real_time_loop.py: simplified loop; not production-ready
import time
from rt_sched import wait_next_cycle  # real-time scheduler helper

Kp = 50.0      # joint stiffness (Nm/rad)
Kd = 5.0       # joint damping (Nms/rad)

def control_loop():
    while True:
        t0 = time.time()
        q, qd = read_joint_state()          # position, velocity sensors
        f_ext = read_force_sensors()        # end-effector force estimate
        q_des, qd_des = planner.get_setpoint()
        f_des = planner.get_desired_force()

        # impedance law (eq. 2) -> compute torques
        tau_task = jacobian_transpose(q) @ f_des            # task-space term
        tau_feedback = Kp*(q_des - q) + Kd*(qd_des - qd)   # joint feedback
        tau = tau_task + tau_feedback

        # safety checks: torque and position limits
        if any(abs(tau) > torque_limits):
            tau = saturate(tau)           # clamp to safe values
            trigger_alert("torque_limit")

        send_torques(tau)                 # actuator interface (real-time)
        wait_next_cycle(t0)               # maintain loop period reliably