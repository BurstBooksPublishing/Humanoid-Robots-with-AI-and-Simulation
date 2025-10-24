# robots: list of robot IDs, tasks: list of task IDs
# local_cost(robot,task) -> float; send/recv use existing comm layer
def auction_round(robots, tasks, comm):
    bids = {}  # task -> (robot, bid)
    for r in robots:
        for t in tasks:
            c = local_cost(r, t)               # local estimate (time+energy+risk)
            if not capable(r, t): continue    # capability check
            # send bid to neighbors (non-blocking)
            comm.broadcast({'type':'bid','robot':r,'task':t,'cost':c})
    # collect bids for fixed time window
    messages = comm.collect(timeout=0.2)
    for m in messages:
        if m['type']=='bid':
            t = m['task']; r = m['robot']; c = m['cost']
            if t not in bids or c < bids[t][1]:
                bids[t] = (r, c)   # lowest-cost wins
    # winners accept; inform network
    assignments = {}
    for t,(r,c) in bids.items():
        comm.broadcast({'type':'assign','task':t,'robot':r})
        assignments[t] = r
    return assignments  # local view; conflicts resolved by protocol rules