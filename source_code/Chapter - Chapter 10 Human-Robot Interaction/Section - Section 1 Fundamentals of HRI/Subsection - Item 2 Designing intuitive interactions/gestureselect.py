import numpy as np

# gesture_templates: list of dicts with keys 'traj' (Nx2), 'modality', 'safety'
# user_prior: dict mapping goal->prior probability
# goal_models: dict mapping goal->function that returns P(traj|goal)
def score_gestures(gesture_templates, partial_obs, user_prior, goal_models):
    scores = []
    for g in gesture_templates:
        traj = g['traj']  # template trajectory (array)
        # compute likelihood of observed partial path given each potential goal
        # here we use goal_models to evaluate P(partial_obs | goal)
        legibility = 0.0
        for goal, prior in user_prior.items():
            likelihood = goal_models[goal](partial_obs)  # model returns scalar
            # posterior proportional to likelihood*prior
            legibility += likelihood * prior
        # cognitive cost heuristic by modality
        modality_cost = {'speech':1.5, 'gesture':1.0, 'display':0.8}[g['modality']]
        # safety score: higher is better; clip to [0,1]
        safety_score = np.clip(g['safety'], 0.0, 1.0)
        # utility: combine (higher legibility and safety preferred; lower cost preferred)
        utility =  w_leg*legibility + w_safe*safety_score - w_cost*modality_cost
        scores.append((utility, g))
    # softmax selection probabilities
    utilities = np.array([u for u,_ in scores])
    probs = np.exp(temperature * (utilities - utilities.max()))
    probs /= probs.sum()
    idx = np.random.choice(len(scores), p=probs)
    return scores[idx][1]  # return selected gesture template

# hyperparameters (tunable via HRI tests)
w_leg, w_safe, w_cost = 1.0, 1.2, 0.8
temperature = 2.0