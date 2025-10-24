import numpy as np

# Parameters (tunable)
log_odds_prior = 0.0          # prior
log_odds_occ = 0.85           # inverse sensor model for hit
log_odds_free = -0.4          # inverse sensor model for miss
log_odds_min, log_odds_max = -4.0, 4.0  # clamping

def update_voxel_logodds(grid, ray_voxels, hit):
    # grid: 3D numpy array of log-odds
    # ray_voxels: list of voxel indices along ray (excluding endpoint)
    # hit: boolean, True if ray ended in an occupied cell
    for idx in ray_voxels:
        grid[idx] += log_odds_free - log_odds_prior   # free-space update
        grid[idx] = np.clip(grid[idx], log_odds_min, log_odds_max)
    if hit:
        end = ray_voxels[-1]
        grid[end] += log_odds_occ - log_odds_prior   # occupied endpoint
        grid[end] = np.clip(grid[end], log_odds_min, log_odds_max)
# Note: ray tracing omitted; use voxel traversal (Amanatides-Woo) in practice.