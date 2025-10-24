class Status: SUCCESS=0; FAILURE=1; RUNNING=2

class Node:
    def tick(self): raise NotImplementedError

class Condition(Node):
    def __init__(self, check): self.check=check
    def tick(self): return Status.SUCCESS if self.check() else Status.FAILURE

class Action(Node):
    def __init__(self, start, update): self.start=start; self.update=update; self.started=False
    def tick(self):
        if not self.started:
            self.start()            # send command to controller
            self.started=True
        return self.update()        # poll status from controller

class Sequence(Node):
    def __init__(self, children): self.children=children; self.index=0
    def tick(self):
        while self.index < len(self.children):
            s = self.children[self.index].tick()
            if s == Status.SUCCESS:
                self.index+=1; continue
            if s == Status.RUNNING: return Status.RUNNING
            self.index=0; return Status.FAILURE
        self.index=0; return Status.SUCCESS

# Example usage: pick object if visible and balanced
# check_visibility, imu_balance_check are sensor-backed functions
# plan_trajectory, execute_motion, close_gripper wrap ROS actions
tree = Sequence([
    Condition(check_visibility),      # quick cheap check
    Condition(imu_balance_check),     # stabilize before motion
    Action(plan_trajectory.start, plan_trajectory.update),
    Action(execute_motion.start, execute_motion.update),
    Action(close_gripper.start, close_gripper.update),
])