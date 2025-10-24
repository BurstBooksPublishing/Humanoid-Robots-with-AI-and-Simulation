import numpy as np
from scipy.optimize import least_squares
# x: [camera_intrinsics(4), cam_tx, cam_ty, cam_tz, quat_cam(4), imu_bias(3)]
def reprojection_residuals(x, observations):
    K = np.array([[x[0], 0, x[2]],[0, x[1], x[3]],[0,0,1]]) # fx,fy,cx,cy
    t_cam = x[4:7]  # camera translation in body frame
    q_cam = x[7:11] # camera quaternion in body frame
    # compute reprojection residuals for each detected landmark
    res = []
    for obs in observations['images']:
        X_body = obs['X_body']  # landmark in body frame (from kinematics/marker)
        u_meas = obs['u']       # image measurement
        # transform to camera frame (apply quaternion rotation) -- placeholder
        X_cam = quat_rotate(q_cam, X_body - t_cam)  # small helper (not shown)
        u_pred = (K @ (X_cam / X_cam[2]))[:2]
        res.extend(u_meas - u_pred)
    # IMU orientation residuals (small-angle via rotation matrices)
    for imu_obs in observations['imu']:
        R_pred = imu_obs['R_pred']   # from kinematics
        R_imu  = imu_obs['R_imu']    # from IMU integration
        # log map of R_pred^T * R_imu -> angle-axis (3-vector)
        res.extend(logSO3(R_pred.T @ R_imu))
    return np.array(res)

# run optimizer (observations prepared separately)
x0 = np.zeros(11)  # initial guess
sol = least_squares(reprojection_residuals, x0, args=(observations,), verbose=2)
# sol.x contains estimated parameters