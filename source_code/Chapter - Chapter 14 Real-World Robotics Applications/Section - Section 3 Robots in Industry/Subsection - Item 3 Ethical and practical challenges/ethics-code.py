class SafetyMonitor:
    def __init__(self, d_min, tau_max, risk_threshold):
        self.d_min = d_min             # minimum allowed distance (m)
        self.tau_max = tau_max         # per-joint torque limits (N*m)
        self.risk_threshold = risk_threshold

    def compute_risk(self, p_robot, p_human, taus, p_fail, severity):
        d = np.linalg.norm(p_robot - p_human)        # distance metric
        # distance factor: grows as d -> d_min
        dist_factor = max(0.0, (self.d_min - d) / self.d_min)
        # torque factor: fraction of joints near limit
        torque_factor = np.mean(np.abs(taus) / self.tau_max)
        # combined risk estimator (simple heuristic)
        risk = p_fail * severity * (1.0 + dist_factor + torque_factor)
        return risk

    def check_and_act(self, state):
        # state contains p_robot, p_human, taus, p_fail, severity
        risk = self.compute_risk(state.p_robot, state.p_human,
                                 state.taus, state.p_fail, state.severity)
        if risk > self.risk_threshold:
            self.emergency_stop()   # immediate actuation halt
        else:
            self.allow_motion()

    def emergency_stop(self):
        # send stop to low-level controller; log event for forensics
        publish(\lstinline|/safety/emergency_stop|, True)  # ROS topic