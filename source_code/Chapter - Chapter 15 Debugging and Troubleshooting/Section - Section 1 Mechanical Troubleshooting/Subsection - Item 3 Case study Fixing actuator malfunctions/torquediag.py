import time, numpy as np
from ros_interface import CommandTorque, ReadState  # hypothetical API

K_T = 0.08  # Nm/A torque constant (example)
SAMPLE_HZ = 1000
steps = [0.0, 2.0, -2.0, 0.0]  # Nm torque steps

log = []
for tau in steps:
    CommandTorque('right_knee', tau)  # command torque (software-limited)
    t0 = time.time()
    while time.time() - t0 < 2.0:  # hold for 2 seconds
        st = ReadState('right_knee')  # returns dict with 'current','pos','vel'
        est_tau = K_T * st['current']  # estimate motor torque
        log.append((time.time(), tau, st['pos'], st['vel'], st['current'], est_tau))
        time.sleep(1.0 / SAMPLE_HZ)
CommandTorque('right_knee', 0.0)  # safe stop
# Post-process: compute steady-state current vs commanded torque to estimate friction.