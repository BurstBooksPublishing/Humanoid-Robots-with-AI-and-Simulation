import numpy as np

def score_candidates(height_map, cov_map, robot_model, params):
    # height_map: 2D array of heights; cov_map: same shape uncertainties
    candidates = []
    for (i,j), h in np.ndenumerate(height_map):
        x,y = map_index_to_world(i,j)                        # world coords
        n = estimate_normal(height_map,i,j)                  # local normal
        theta = np.arccos(np.dot(n, [0,0,1]))                # slope
        r = local_roughness(height_map,i,j)                  # variance
        reachable, d_ik = robot_model.ik_reachability(x,y,h) # IK check
        zmp_pen = predict_zmp_violation(robot_model, x,y,h)  # predicted ZMP distance
        uncert = np.trace(cov_map[i,j])                      # uncertainty penalty
        cost = (params['ws']*theta + params['wr']*r + params['wk']*d_ik
                + params['wz']*max(0, zmp_pen) + params['wu']*uncert)
        if reachable:
            candidates.append({'pos':(x,y,h),'cost':cost})
    # return sorted candidates
    return sorted(candidates, key=lambda c: c['cost'])
# Note: functions map_index_to_world, estimate_normal, robot_model.* are implemented elsewhere.