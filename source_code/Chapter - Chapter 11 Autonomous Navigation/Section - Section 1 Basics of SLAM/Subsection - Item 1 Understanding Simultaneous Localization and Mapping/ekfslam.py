import numpy as np
# state: [px,py,pz,qx,qy,qz,qw, m1x,m1y,...] ; P covariance
def predict(state, P, imu, dt, Q): 
    # propagate pose with IMU accel/gyro (simple integration) -- use preintegration in real code
    pose = state[:7]
    # ... integrate to get new_pose (placeholder)
    new_pose = integrate_pose(pose, imu, dt)  # // integrate on SE(3)
    F = compute_F(pose, imu, dt)              # // state jacobian
    state[:7] = new_pose
    P = F @ P @ F.T + Q
    return state, P

def update(state, P, z, R, landmark_idx):
    # measurement model returns expected bearing-range to landmark
    h, H = measurement_model(state, landmark_idx)  # // H jacobian
    y = z - h
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    state = state + K @ y
    P = (np.eye(P.shape[0]) - K @ H) @ P
    return state, P

# main loop (pseudo)
state, P = init_state(), init_cov()
for imu, measurements, dt in sensor_stream():   # // sensor_stream from simulation or hardware
    state, P = predict(state, P, imu, dt, Q_imu)
    for z, idx in measurements:                  # // features observed this timestep
        state, P = update(state, P, z, R_feat, idx)
    # occasional loop closure triggers graph optimization (not shown)