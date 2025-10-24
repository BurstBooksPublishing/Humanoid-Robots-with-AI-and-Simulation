# Simple fleet manager: assign each incoming task to robot with min ETA.
import time
from math import ceil

def estimate_completion(robot_state, task):  # returns ETA in seconds
    # estimate travel + pick + place times; include battery swap if needed
    return robot_state['eta'] + task['travel'] + task['pick'] + task['place']

def assign_tasks(task_queue, robots):  # robots: list of dict states
    assignments = []
    while task_queue:
        task = task_queue.pop(0)                      # get next task (FIFO)
        best_robot = min(robots, key=lambda r: estimate_completion(r, task))
        best_robot['eta'] = estimate_completion(best_robot, task)  # update ETA
        best_robot['tasks'].append(task)                           # record
        assignments.append((task, best_robot['id']))
    return assignments

# Example usage: run periodically; monitor robot battery and reassign on faults.