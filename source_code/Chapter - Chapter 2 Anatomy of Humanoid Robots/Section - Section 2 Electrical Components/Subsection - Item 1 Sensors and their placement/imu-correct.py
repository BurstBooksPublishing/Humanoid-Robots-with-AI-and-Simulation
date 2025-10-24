import numpy as np

def accel_correct(a_com, alpha, omega, r):  # a_com, alpha, omega, r are 3-vectors
    # coriolis and centripetal terms due to offset r
    coriolis = np.cross(alpha, r)            # alpha x r
    centripetal = np.cross(omega, np.cross(omega, r))  # omega x (omega x r)
    a_sensor = a_com + coriolis + centripetal         # ideal sensor accel (no gravity)
    return a_sensor

# Example usage: compute expected accelerometer reading at wrist IMU.
# a_com measured from trunk estimator; alpha, omega from joint kinematics; r from CAD.