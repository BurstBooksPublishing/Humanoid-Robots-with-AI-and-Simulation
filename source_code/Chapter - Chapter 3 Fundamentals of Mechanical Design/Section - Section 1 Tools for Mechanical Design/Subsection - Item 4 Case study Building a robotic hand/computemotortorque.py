import math
# Finger geometry (meters)
l1, l2 = 0.05, 0.03
# Joint angles (radians)
theta1, theta2 = math.radians(30), math.radians(20)
# Desired fingertip force normal to finger (Newtons)
F_tip = 10.0
# Fingertip force vector in world frame (px, py) for normal grasp
Fx, Fy = 0.0, F_tip

# Jacobian for planar two-link finger
J11 = -l1*math.sin(theta1) - l2*math.sin(theta1+theta2)
J12 = -l2*math.sin(theta1+theta2)
J21 =  l1*math.cos(theta1) + l2*math.cos(theta1+theta2)
J22 =  l2*math.cos(theta1+theta2)
J = [[J11, J12], [J21, J22]]

# Compute joint torques tau = J^T * F
tau1 = J11*Fx + J21*Fy
tau2 = J12*Fx + J22*Fy

# Transmission and motor parameters
gear_ratio = 50.0      # typical compact gearbox
efficiency = 0.85
motor_tau1 = abs(tau1) / (gear_ratio * efficiency)  # required motor torque (Nm)
motor_tau2 = abs(tau2) / (gear_ratio * efficiency)

print("# Joint torques (Nm):", tau1, tau2)           # torque at joints
print("# Motor torques (Nm):", motor_tau1, motor_tau2) # torque at motors