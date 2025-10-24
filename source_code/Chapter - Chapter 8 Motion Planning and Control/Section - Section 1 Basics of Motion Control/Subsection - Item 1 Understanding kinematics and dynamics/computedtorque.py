import numpy as np
import pinocchio as pin

# model, data created at initialization
model = pin.buildModelFromUrdf("humanoid.urdf")
data = model.createData()
robot = pin.RobotWrapper(model)  # wrapper for kinematics/dynamics

Kp = np.diag([...])  # proportional gains per joint
Kd = np.diag([...])  # derivative gains per joint

def control_step(q, qdot, q_des, qdot_des, qdd_des, f_contact_est):
    # forward kinematics & dynamics (fills data)
    pin.forwardKinematics(model, data, q, qdot)
    pin.computeAllTerms(model, data, q, qdot)  # computes M,C,g
    M = data.M
    C = pin.computeCoriolis(model, data, q, qdot)  # library helper
    g = data.gravity
    # desired joint-space acceleration with PD feedback
    acc_cmd = qdd_des + Kd.dot(qdot_des - qdot) + Kp.dot(q_des - q)
    # compute torques (computed torque + contact compensation)
    tau = M.dot(acc_cmd) + C.dot(qdot) + g - pin.jacobianContactTerm(model, data, q, f_contact_est)
    # saturate torques to actuator limits before sending
    tau = np.clip(tau, -torque_limits, torque_limits)
    return tau