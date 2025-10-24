robots = [
    {"name":"WABOT-1","year":1973,"dof":17,"act":"electric"},      # early full-body research robot
    {"name":"Unimate","year":1961,"dof":6,"act":"electric"},      # first industrial arm
    {"name":"Honda_ASIMO","year":2000,"dof":34,"act":"electric"}, # integrated locomotion and manipulation
    {"name":"Boston_Atlas","year":2013,"dof":28,"act":"hydraulic"},# dynamic agility focus
]
# Compute simple state vector size (joint angles + velocities)
for r in robots:
    n = r["dof"]
    state_size = 2 * n  # q and q_dot
    print(f"{r['name']} ({r['year']}): state_size={state_size}")  # engineer-facing summary