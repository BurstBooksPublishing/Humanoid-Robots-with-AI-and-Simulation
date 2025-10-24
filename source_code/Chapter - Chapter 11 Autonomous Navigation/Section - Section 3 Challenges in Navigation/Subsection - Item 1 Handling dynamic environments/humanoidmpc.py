import numpy as np
# simple constant-velocity predictor (states: [x,y,vx,vy])
def predict_obstacle(state, P, dt, steps):
    F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
    Q = 0.01*np.eye(4)  # process noise
    preds, Ps = [], []
    x = state.copy(); Pk = P.copy()
    for _ in range(steps):
        x = F.dot(x)
        Pk = F.dot(Pk).dot(F.T) + Q
        preds.append(x[:2].copy())  # predicted position
        Ps.append(Pk[:2,:2].copy()) # position covariance
    return preds, Ps

def compute_safe_distance(P, base_margin=0.6):
    # inflate margin using largest eigenvalue of covariance
    eig = np.linalg.eigvals(P)
    sigma = np.sqrt(max(eig.real, 1e-6))
    return base_margin + 2.0*sigma   # 95% approx

def mpc_plan(current_state, predicted_obs, safe_dist):
    # placeholder: call to a real MPC (CasADi/CVX) goes here
    # returns sequence of footstep targets and CoM trajectory
    return {"footsteps": [(0.5,0.0)], "com": [(0,0,0.9)]}

# main loop (simplified)
obs_state = np.array([2.0, 0.5, -0.2, 0.0])
P = np.eye(4)*0.05
preds, Ps = predict_obstacle(obs_state, P, dt=0.1, steps=10)
safe = compute_safe_distance(Ps[0])
plan = mpc_plan(current_state=None, predicted_obs=preds, safe_dist=safe)
# apply first controls from plan (actuation layer handles low-level tracking)