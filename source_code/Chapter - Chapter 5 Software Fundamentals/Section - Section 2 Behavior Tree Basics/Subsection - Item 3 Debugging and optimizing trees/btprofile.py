import time, logging
# node API: node.tick() -> Status, node.name
TIMEOUT=0.05  # seconds per node budget
stats = {}     # accumulate counts and times

def profile_tick(node):
    start = time.perf_counter()
    status = node.tick()              # actual node execution
    elapsed = time.perf_counter() - start

    # update stats
    s = stats.setdefault(node.name, {'count':0,'total':0.0,'max':0.0,'succ':0})
    s['count'] += 1
    s['total'] += elapsed
    s['max'] = max(s['max'], elapsed)
    if status == 'SUCCESS': s['succ'] += 1

    # timeout safeguard
    if elapsed > TIMEOUT:
        logging.warning(f"Node {node.name} exceeded timeout {elapsed:.3f}s")
        node.abort()                  # safe abort or fallback action

    return status

# integration: replace tree.tick() to call profile_tick on each node.