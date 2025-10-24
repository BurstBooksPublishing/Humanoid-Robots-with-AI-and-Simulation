# realtime loop: compute desired torque from Cartesian impedance
import numpy as np
dt = 0.001  # 1 kHz loop
Kd = np.diag([2000,2000,2000,50,50,50])  # stiffness (N/m, N·m/rad)
Bd = np.diag([50,50,50,5,5,5])           # damping
Md = np.diag([1.0,1.0,1.0,0.1,0.1,0.1])  # virtual inertia

while True:
    x, xd, xd_dot = read_cartesian_state()         # current, desired pos/vel
    Fext = read_force_torque()                     # measured external wrench
    e = x - xd
    edot = xd_dot - get_cartesian_velocity()
    # desired Cartesian wrench from impedance law (discrete approx)
    Fdes = Md @ ( -edot/dt ) + Bd @ edot + Kd @ e
    # safety clamp on force magnitude (prevent tissue damage)
    if np.linalg.norm(Fext) > 30.0:                 # 30 N safety threshold
        Fdes = np.zeros(6)
        trigger_emergency_stop()                    # immediate safe stop
    # map desired wrench to joint torques
    J = compute_jacobian()
    tau = J.T @ Fdes + compute_nullspace_torque()   # add posture control
    send_torque_command(tau)                        # low-level actuator interface
    sleep(dt)