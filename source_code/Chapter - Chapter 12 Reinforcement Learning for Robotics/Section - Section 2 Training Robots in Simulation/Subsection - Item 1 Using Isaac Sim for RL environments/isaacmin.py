import numpy as np
from omni.isaac.core import World  # main simulator object
from omni.isaac.core.articulations import Articulation  # load humanoid
# create world and load humanoid USD
world = World(stage_units_in_meters=0.001)
humanoid = Articulation("/World/Humanoid", usd_path="humanoid.usd")
world.scene.add(humanoid)

class HumanoidEnv:
    def __init__(self, world, humanoid): 
        self.world = world
        self.humanoid = humanoid
        self.dt = 0.016  # control timestep
        # PD gains for inner loop
        self.Kp = np.array([100.0]*humanoid.num_joints)
        self.Kd = np.array([2.0]*humanoid.num_joints)

    def reset(self):
        # reset pose and velocities; return observation
        self.humanoid.set_local_root_pose(np.zeros(6))  # base pose
        self.humanoid.set_joint_positions(np.zeros(self.humanoid.num_joints))
        self.humanoid.set_joint_velocities(np.zeros(self.humanoid.num_joints))
        self.world.reset()
        return self._get_obs()

    def step(self, action):
        # action are desired joint positions; compute PD torques
        q = self.humanoid.get_joint_positions()
        qd = self.humanoid.get_joint_velocities()
        tau = self.Kp*(action - q) - self.Kd*qd  # PD policy output
        self.humanoid.apply_torques(tau)  # apply to simulator
        self.world.step()  # advance simulator by self.dt
        obs = self._get_obs()
        reward = self._compute_reward(obs, action)
        done = self._check_termination(obs)
        return obs, reward, done, {}

    def _get_obs(self):
        return np.concatenate([self.humanoid.get_joint_positions(),
                               self.humanoid.get_joint_velocities()])

    def _compute_reward(self, obs, action):
        # simple upright penalty and energy penalty
        upright = 1.0 - np.abs(obs[0])  # base roll approx (placeholder)
        energy = -0.001 * np.sum(np.square(action))  # penalize large commands
        return upright + energy