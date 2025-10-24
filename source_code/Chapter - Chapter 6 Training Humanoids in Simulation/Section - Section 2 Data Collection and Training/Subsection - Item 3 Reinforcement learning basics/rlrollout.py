# assume env is a vectorized Isaac Sim environment; policy is a PyTorch module
# collect a batch of rollouts
obs_batch, actions_batch, rewards_batch, dones_batch, logp_batch, vals_batch = [], [], [], [], [], []
obs = env.reset()
for t in range(T):                     # T: rollout length
    action, logp, value = policy.step(obs)  # sample action; get log-prob and value
    next_obs, reward, done, info = env.step(action)
    obs_batch.append(obs); actions_batch.append(action)
    rewards_batch.append(reward); dones_batch.append(done)
    logp_batch.append(logp); vals_batch.append(value)
    obs = next_obs

# compute returns and GAE advantages (vectorized)
returns = compute_returns(rewards_batch, vals_batch, dones_batch, gamma=0.99)
advantages = returns - vals_batch

# single gradient update (vanilla policy gradient with baseline)
optimizer.zero_grad()
policy_loss = - (logp_batch * advantages.detach()).mean()           # maximize expected advantage
value_loss = 0.5 * (returns - vals_batch).pow(2).mean()             # critic MSE
entropy_loss = -policy.entropy().mean()                             # encourage exploration
loss = policy_loss + 0.5 * value_loss + 0.01 * entropy_loss
loss.backward(); optimizer.step()