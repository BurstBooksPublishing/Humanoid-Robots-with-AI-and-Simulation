class HybridArbiterNode(BTNode):
    def __init__(self, params):
        super().__init__(params)
        self.prev_action = None
        self.weights = {'bt':1.0, 'rl':0.9, 'mpc':1.2}
        self.lambda_switch = 0.5

    def tick(self, state):                         # state: sensor dict
        candidates = []                            # collect (action,source,utility)
        # Query BT leaf (fast, symbolic) -- returns discrete action id
        a_bt, u_bt, v_bt = query_bt_leaf(state)    # v_bt validity boolean
        if v_bt: candidates.append((a_bt,'bt',u_bt))
        # Query RL policy (continuous) -- returns action id + confidence
        a_rl, u_rl, v_rl = query_rl_policy(state)
        if v_rl: candidates.append((a_rl,'rl',u_rl))
        # Query MPC/WBC (stability-focused) -- returns action id + utility
        a_mpc, u_mpc, v_mpc = query_mpc(state)
        if v_mpc: candidates.append((a_mpc,'mpc',u_mpc))
        # Filter by safety monitor
        safe = [c for c in candidates if safety_check(state,c[0])]
        if not safe: return self.fail()             # no safe actions
        # Compute score with switching penalty
        def score(c):
            a,src,u = c
            w = self.weights[src]
            switch = 0 if (self.prev_action == a) else 1
            return w*u - self.lambda_switch*switch
        chosen = max(safe, key=score)
        self.prev_action = chosen[0]
        dispatch_action(chosen[0])                  # send to controllers
        return self.success()