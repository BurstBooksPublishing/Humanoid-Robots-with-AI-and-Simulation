import numpy as np, math

# Robot params
r = 0.35              # robot radius (m)
sigma = 0.15          # position std dev (m)
lambda_s, lambda_e, lambda_d = 5.0, 1.0, 0.5  # weights

def normal_cdf(x):
    return 0.5 * (1 + math.erf(x / math.sqrt(2)))  # standard normal CDF

def collision_prob(d_min, r=r, sigma=sigma):
    # Probability of collision for isotropic Gaussian position error.
    return 1.0 - normal_cdf((d_min - r) / sigma)

def energy_estimate(curr, goal):
    # Simplified energy proxy: Euclidean distance times payload factor.
    return np.linalg.norm(goal - curr) * 10.0  # J proxy

def score_waypoint(curr, wp, d_min):
    p_coll = collision_prob(d_min)               # safety term
    e_cost = energy_estimate(curr, wp)           # energy term
    dist = np.linalg.norm(wp - curr)             # progress term
    return lambda_s * p_coll + lambda_e * e_cost + lambda_d * dist

# Example candidates
curr = np.array([0.0, 0.0])
candidates = [np.array([x, y]) for x,y in [(1,0),(0.8,0.3),(0.5,0.9)]]
d_mins = [0.6, 0.4, 0.8]  # clearance from local map
best = min(zip(candidates, d_mins), key=lambda cd: score_waypoint(curr, cd[0], cd[1]))
# best[0] is chosen waypoint