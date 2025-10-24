from math import ceil
from enum import Enum

# Status enum
class Status(Enum):
    SUCCESS = 1
    RUNNING = 2
    FAILURE = 3

# Base node
class Node:
    def tick(self, blackboard): raise NotImplementedError

# Parallel node with k-of-n success threshold
class Parallel(Node):
    def __init__(self, children, k=None, p=None):
        self.children = children
        n = len(children)
        self.k = k if k is not None else (ceil(p*n) if p else n)
    def tick(self, blackboard):
        successes = 0
        for c in self.children:
            s = c.tick(blackboard)
            if s == Status.SUCCESS: successes += 1
        return Status.SUCCESS if successes >= self.k else Status.FAILURE

# Simple sequence
class Sequence(Node):
    def __init__(self, children): self.children = children
    def tick(self, blackboard):
        for c in self.children:
            s = c.tick(blackboard)
            if s != Status.SUCCESS: return s
        return Status.SUCCESS

# Dynamic subtree injection example
class DynamicManager(Node):
    def __init__(self): self.root = Sequence([])  # placeholder
    def inject(self, subtree): self.root.children.append(subtree)  # inject task
    def remove(self, subtree): self.root.children.remove(subtree)
    def tick(self, blackboard): return self.root.tick(blackboard)

# Example atomic behaviors for humanoid
class CheckBalance(Node):
    def tick(self, bb):
        return Status.SUCCESS if bb.get('stability')>0.5 else Status.FAILURE

class PlanFootstep(Node):
    def tick(self, bb):
        return Status.RUNNING  # planner may require multiple ticks

# Usage
blackboard = {'stability': 0.6}
p = Parallel([CheckBalance(), CheckBalance(), CheckBalance()], p=0.66)  # require ~2/3
mgr = DynamicManager()
mgr.inject(p)
mgr.inject(PlanFootstep())  # add planning subtree