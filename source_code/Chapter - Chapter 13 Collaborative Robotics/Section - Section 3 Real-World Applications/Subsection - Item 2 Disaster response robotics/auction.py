import numpy as np
# assume grid_prob: 2D array of p_v; robot_pose and tasks list provided
def info_gain(grid_prob, region_idx):
    p = grid_prob[region_idx]                      # local subgrid probability
    H_before = -np.sum(p * np.log(np.clip(p,1e-9,1)))
    # assume sensor will reduce entropy by a factor gamma_sensor
    gamma_sensor = 0.5
    H_after = H_before * (1 - gamma_sensor)
    return H_before - H_after

def energy_cost(robot_pose, region_centroid):
    d = np.linalg.norm(np.array(robot_pose[:2]) - np.array(region_centroid[:2]))
    speed = 0.5                                     # m/s nominal
    return d / speed                                # time proxy

def compute_bid(robot_pose, grid_prob, region_idx, region_centroid, alpha=1.0,beta=0.7,gamma=1.2):
    I = info_gain(grid_prob, region_idx)
    E = energy_cost(robot_pose, region_centroid)
    R = estimate_risk(region_centroid)              # domain-specific function
    U = alpha*I - beta*E - gamma*R
    return U

# publish bid (compact message) to team channel \lstinline|/team/bids|.
# team nodes perform softmax and assign region.