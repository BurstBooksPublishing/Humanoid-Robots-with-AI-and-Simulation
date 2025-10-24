# Simple safety monitor for humanoid limb approach (HIL-compatible).
class SafetyMonitor:
    def __init__(self, reaction_ms=50, decel=3.0, margin=0.05):
        self.t_react = reaction_ms / 1000.0  # s
        self.a_dec = decel                     # m/s^2
        self.margin = margin                   # m

    def stopping_distance(self, v):
        # kinematic stopping distance per Eq. (1)
        return v * self.t_react + 0.5 * v * v / self.a_dec

    def check_and_act(self, measured_dist, rel_speed, max_force, force_limit):
        s_req = self.stopping_distance(rel_speed) + self.margin
        if measured_dist <= s_req:
            # trigger rated E-Stop and log event
            self.trigger_estop(reason="separation_violation", required=s_req, measured=measured_dist)
            return False
        if max_force > force_limit:
            # soft reflex: cut torque command and log contact
            self.cut_torque()
            self.log_event("force_limit_exceeded", value=max_force)
            return False
        return True

    def trigger_estop(self, **info):
        # HIL: assert safety bus; in hardware connect to safe relay.
        print("E-STOP triggered:", info)

    def cut_torque(self):
        # reduce joint torque commands quickly
        print("Torque cut: entering compliant mode")