import py_trees as pt
import numpy as np

class TaskAllocator(pt.behaviour.Behaviour):
    def __init__(self, tasks, battery_provider): 
        super().__init__("TaskAllocator")
        self.tasks = tasks                # dict: id -> {u,c,t,r,emergency}
        self.battery = battery_provider   # callable -> battery level
        self.active = set()

    def update(self):
        B = self.battery()                # read battery (ROS topic)
        # greedy score/cost ratio, respect actuators and safety
        ordered = sorted(self.tasks.items(), key=lambda kv: kv[1]['u']/kv[1]['c'], reverse=True)
        chosen, used_cost, used_res = set(), 0.0, set()
        for tid, meta in ordered:
            if meta.get('emergency', False):
                chosen = {tid}; break      # preemptive emergency
            if used_cost + meta['c'] > B: continue
            if used_res & set(meta['r']): continue
            if not meta.get('safety_ok', True): continue
            chosen.add(tid); used_cost += meta['c']; used_res |= set(meta['r'])
        self.active = chosen
        # publish allocation (ROS param/service) for decorators to query
        return pt.common.Status.SUCCESS

# Activation decorator consults allocator.active
class ActivationDecorator(pt.decorators.Decorator):
    def __init__(self, name, task_id, allocator, child):
        super().__init__(name=name, child=child)
        self.task_id = task_id; self.allocator = allocator
    def update(self):
        if self.task_id in self.allocator.active:
            return self.decorated.update()   # run child subtree
        else:
            # child receives abort signal via ROS action cancel or internal flag
            return pt.common.Status.FAILURE

# Wiring the tree
allocator = TaskAllocator(tasks_dict, battery_provider)
supervisor = build_supervisor_subtree()  # safety monitors
task_pool = pt.composites.Parallel("TaskPool", policy=pt.common.ParallelPolicy.SuccessOnAll())
for tid, meta in tasks_dict.items():
    leaf = build_task_subtree(tid, meta)   # motion, perception actions
    deco = ActivationDecorator(f"Active_{tid}", tid, allocator, leaf)
    task_pool.add_child(deco)
root = pt.composites.Parallel("Root", policy=pt.common.ParallelPolicy.SuccessOnAll())
root.add_children([allocator, supervisor, task_pool])