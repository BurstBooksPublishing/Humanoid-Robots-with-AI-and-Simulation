import numpy as np

# params
g = 9.81
z_com = 0.85             # nominal CoM height (m)
omega0 = np.sqrt(g/z_com)
alpha = 0.8              # low-pass filter coeff

# state (initialized elsewhere)
zmp_filtered = np.zeros(2)

def compute_zmp(ft_left, ft_right, imu_accel, foot_positions):
    # ft_*: dict with 'force' np.array([Fx,Fy,Fz]) and 'moment' np.array([Mx,My,Mz])
    # foot_positions: dict with 'left' and 'right' contact point in world frame
    # aggregate forces/wrenches
    F = ft_left['force'] + ft_right['force']
    M = ft_left['moment'] + ft_right['moment']
    Fz = F[2] + 1e-6  # avoid division by zero

    # ZMP from total moment about ground (Eq. ZMP)
    x_zmp = -M[1] / Fz
    y_zmp =  M[0] / Fz
    zmp_meas = np.array([x_zmp, y_zmp])

    # low-pass filter to reduce sensor noise
    global zmp_filtered
    zmp_filtered = alpha * zmp_filtered + (1-alpha) * zmp_meas

    # CoM state estimate using IMU acceleration (simple integrator shown for clarity)
    # in practice use an EKF with kinematic priors
    com_pos = np.array([0.0, 0.0])    # replace with estimator
    com_vel = np.array([0.0, 0.0])    # replace with estimator

    # capture point prediction (LIPM)
    cp = com_pos + com_vel / omega0

    return zmp_filtered, cp

# Example call with dummy sensors (replace with ROS topics or drivers)
# zmp, cp = compute_zmp(ft_left, ft_right, imu_accel, foot_positions)