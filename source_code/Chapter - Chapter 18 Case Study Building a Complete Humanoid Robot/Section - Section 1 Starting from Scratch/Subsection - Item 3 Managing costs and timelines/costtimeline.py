import random, math
# Define tasks: (name, a,m,b,cost_normal,cost_crash,crash_days)
tasks = [
  ('Frame', 5, 7, 10, 8000, None, 0),      # weeks, vendor frame
  ('Actuators', 8, 12, 16, 50000, 65000, 0),# long lead, possible premium
  ('PCB', 2, 4, 6, 4000, 7000, 0),          # custom electronics
  ('ControlSW', 6, 10, 14, 60000, None, 0), # integration and control
  ('Integration', 4, 6, 10, 20000, None, 0)
]
# Precedence as adjacency list (simple linear example)
precedence = {'Frame':[], 'Actuators':['Frame'], 'PCB':['Frame'], 
              'ControlSW':['Actuators','PCB'], 'Integration':['ControlSW']}

def sample_duration(a,m,b):
    # PERT beta approx via triangular->use normal approximation mean and sigma
    mean = (a+4*m+b)/6.0
    sigma = (b-a)/6.0
    return max(0.1, random.gauss(mean, sigma))

def critical_path_duration(sampled):
    # simple DAG longest-path by dynamic programming
    order = ['Frame','Actuators','PCB','ControlSW','Integration']
    longest = {}
    for t in order:
        preds = precedence[t]
        start = 0.0
        for p in preds:
            start = max(start, longest[p])
        longest[t] = start + sampled[t]
    return max(longest.values())

# Monte Carlo
N=5000
deadline_weeks = 40
count_on_time = 0
costs = []
for _ in range(N):
    sampled = {name: sample_duration(a,m,b) for (name,a,m,b, *_ ) in tasks}
    dur = critical_path_duration(sampled)
    # crude cost: normal cost plus 10% per week over schedule (penalty)
    base_cost = sum(t[4] for t in tasks)
    penalty = base_cost * 0.10 * max(0, (dur-deadline_weeks)/deadline_weeks)
    total_cost = base_cost + penalty
    costs.append(total_cost)
    if dur <= deadline_weeks: count_on_time += 1

print("P(on time)=", count_on_time/N, "mean cost=", sum(costs)/N)
# Use results to choose crash options with slope analysis (not shown).