class HumanoidEnv:
    def __init__(self, sim):
        self.sim = sim
        # PD gains, torque limits configured here
    def randomize_dynamics(self):
        # randomize mass and friction for robust policies
        self.sim.set_mass_scale(1.0 + 0.1 * np.random.randn())  # short comment
        self.sim.set_friction(0.8 + 0.2 * np.random.rand())
    def step(self, action):
        # action is desired joint positions (low-level PD)
        q_des = action  # desired joint targets
        q, qdot = self.sim.get_proprioception()  # get state
        # simple PD control to compute torques
        tau = Kp * (q_des - q) + Kd * (-qdot)  # compute control torque
        tau = np.clip(tau, -torque_limit, torque_limit)  # enforce limits
        self.sim.apply_torques(tau)  # advance sim
        self.sim.step()  # step physics
        obs = self._compute_observation()  # build observation
        reward, done = self._compute_reward(q, qdot, tau)  # reward and termination
        return obs, reward, done, {}
    def _compute_reward(self, q, qdot, tau):
        # progress, energy penalty, and safety term
        progress = self._forward_velocity_term()
        energy = -np.sum(np.abs(tau)) * 1e-3
        safety = -int(self.sim.is_fallen())
        r = 1.0*progress + 0.1*energy + 2.0*safety
        done = self.sim.is_fallen()
        return r, done