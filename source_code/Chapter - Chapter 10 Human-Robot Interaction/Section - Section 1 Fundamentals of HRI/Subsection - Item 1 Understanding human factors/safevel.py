import math

def safe_velocity(distance, mu_r, sigma_r, a_max, d_human_motion=0.0, k=2.0):
    # distance: measured robot-to-human separation (m)
    # mu_r, sigma_r: reaction-time stats (s)
    # a_max: max deceleration (m/s^2), positive
    # d_human_motion: expected human displacement before reaction (m)
    # k: safety Gaussian multiplier
    # Solve v such that v*(mu_r + k*sigma_r) + v**2/(2*a_max) <= distance - d_human_motion
    margin = max(0.0, distance - d_human_motion)
    if margin <= 0.0:
        return 0.0
    tau = mu_r + k * sigma_r
    # quadratic: (1/(2*a)) v^2 + tau v - margin <= 0
    A = 1.0 / (2.0 * a_max)
    B = tau
    C = -margin
    disc = B*B - 4*A*C
    if disc < 0:
        return 0.0
    v_max = (-B + math.sqrt(disc)) / (2*A)
    return max(0.0, v_max)

# Example usage: safe_velocity(1.2, 0.5, 0.1, 2.0) -> speed limit in m/s