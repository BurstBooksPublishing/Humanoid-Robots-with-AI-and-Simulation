def select_next_waypoint(state, waypoints, battery_joules):
    # state: robot pose and velocity
    # waypoints: list of (pose, estimated_traverse_cost_J)
    # battery_joules: remaining energy
    safe_margin = 0.15 * battery_joules  # reserve 15% energy
    candidates = []
    for wp, cost in waypoints:
        # include compute and comm overhead estimate
        overhead = estimate_overhead(wp)  # sensor+comm cost (J)
        total_cost = cost + overhead
        if total_cost + safe_margin <= battery_joules:
            candidates.append((wp, total_cost))
    if not candidates:
        return None  # hold or request remote instructions
    # choose minimal total_cost weighted by scientific value
    return min(candidates, key=lambda x: x[1])[0]