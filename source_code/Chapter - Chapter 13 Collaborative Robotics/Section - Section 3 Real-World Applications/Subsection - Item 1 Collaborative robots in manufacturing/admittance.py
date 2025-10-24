# Admittance controller: M*v_dot + B*v = F_ext - K*x  (discrete Euler integration)
M = 0.5            # virtual mass (kg)
B = 50.0           # damping (N s/m)
K = 200.0          # stiffness after grip (N/m)
dt = 0.01          # control timestep (s)
v = 0.0            # current commanded velocity (m/s)
x = 0.0            # relative displacement (m)

def read_force_sensor():
    # return scalar force along approach axis (N) -- replace with real sensor read
    return get_wrist_force()

def safety_check(force, pose_uncertainty):
    # bound velocity from eq. (3)
    v_nominal = 0.4
    vmax = min(v_nominal, max(0.05, F_MAX / (B + 10.0*pose_uncertainty)))
    return vmax

while robot_enabled():
    F_ext = read_force_sensor()
    pose_unc = get_pose_uncertainty()     # confidence from perception
    vmax = safety_check(F_ext, pose_unc)
    # simple admittance update (explicit)
    v_dot = (F_ext - K*x - B*v) / M
    v = v + v_dot * dt
    # saturate commanded velocity
    v = max(min(v, vmax), -vmax)
    x = x + v * dt                        # integrate displacement
    send_cartesian_velocity(v)            # command robot; safety layer monitors this
    sleep(dt)