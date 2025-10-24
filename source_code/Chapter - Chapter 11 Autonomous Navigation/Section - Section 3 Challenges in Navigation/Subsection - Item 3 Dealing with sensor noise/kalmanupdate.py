def kalman_measurement_update(x, P, z, H, R, gate_thresh=9.21):
    # predict measurement and innovation
    z_pred = H @ x
    y = z - z_pred                      # innovation
    S = H @ P @ H.T + R                 # innovation covariance
    maha = float(y.T @ np.linalg.inv(S) @ y)  # Mahalanobis distance
    if maha > gate_thresh:
        # reject outlier; inflate R to downweight or skip update
        R = R * 1e3                     # conservative fallback (adaptive)
        S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)      # Kalman gain
    x = x + K @ y                       # state update
    P = (np.eye(len(x)) - K @ H) @ P    # covariance update
    return x, P
# Comments: gate_thresh 9.21 approximates 95% for 2-DoF measurement.