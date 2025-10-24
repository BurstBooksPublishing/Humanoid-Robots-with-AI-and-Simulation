import gym
from stable_baselines3 import PPO
# env: Isaac Sim wrapper that returns dict observations and applies torque limits
env = gym.make('IsaacHumanoidBalance-v0')  # custom environment

def custom_reward(obs, action, next_obs):
    # uprightness: negative pitch-roll magnitude
    theta = next_obs['torso_angles'][:2]  # roll, pitch
    r_u = 5.0 * np.exp(-2.0 * np.linalg.norm(theta))
    # energy cost: torque * joint_velocity
    r_e = -0.01 * np.sum(np.abs(action * next_obs['joint_vel']))
    # jerk penalty approximated by action change (requires stateful storage)
    r_j = -0.005 * np.sum(np.abs(action - env.prev_action))
    # fall penalty
    r_f = -50.0 if next_obs['fallen'] else 0.0
    return r_u + r_e + r_j + r_f

# wrap env to override reward with custom_reward
class RewardWrapper(gym.Wrapper):
    def step(self, a):
        obs, base_reward, done, info = self.env.step(a)
        reward = custom_reward(obs, a, obs)
        self.env.prev_action = a.copy()
        return obs, reward, done, info

wenv = RewardWrapper(env)
model = PPO("MlpPolicy", wenv, verbose=1, tensorboard_log="./ppo_balance_tb")
model.learn(total_timesteps=5_000_000)  # realistic budgets are higher
model.save("ppo_humanoid_balance")  # save trained policy