import numpy as np

class HumanoidEKF:
    def __init__(self, x0, P0, Q, R_pose):
        self.x = x0              # state: [px,py,pz, vx,vy,vz, bax, bay, baz]
        self.P = P0
        self.Q = Q
        self.R_pose = R_pose

    def predict(self, imu_omega, imu_accel, dt):
        # simple constant-accel model with accel corrected by biases
        p = self.x[0:3]; v = self.x[3:6]; b_a = self.x[6:9]
        acc = imu_accel - b_a
        p_new = p + v*dt + 0.5*acc*dt*dt
        v_new = v + acc*dt
        self.x[0:3] = p_new; self.x[3:6] = v_new
        # linearized F and process noise Q propagate covariance
        F = np.eye(len(self.x))
        F[0:3,3:6] = np.eye(3)*dt
        G = np.zeros_like(self.x); G[3:6] = np.ones(3)*dt
        self.P = F @ self.P @ F.T + self.Q

    def update_pose(self, pose_meas, H_pose):
        # pose_meas is [px,py,pz]; H_pose maps state to pose
        z = pose_meas
        h = H_pose @ self.x
        S = H_pose @ self.P @ H_pose.T + self.R_pose
        K = self.P @ H_pose.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - h)
        self.P = (np.eye(len(self.x)) - K @ H_pose) @ self.P

# usage: initialize and feed imu at high rate and pose updates when available