# collect batch of transitions from N agents (obs, actions, rewards, next_obs, dones)
# obs: list of agent observations; actions: list of agent actions
# centralized_state aggregates environment state for critic
# critic: Q_phi, value: V_psi, policies: pi_theta[i]

# compute targets for critic (TD(0) for clarity)
with torch.no_grad():
    next_actions = [pi(next_obs[i]).sample() for i, pi in enumerate(policies)]  # decentralized sampling
    q_next = critic(centralized_next_state, torch.cat(next_actions, dim=-1))
    target_q = rewards.sum(dim=1) + gamma * (1.0 - dones.float()) * q_next

# critic loss and update
q_pred = critic(centralized_state, torch.cat(actions, dim=-1))
critic_loss = F.mse_loss(q_pred, target_q)
critic_optimizer.zero_grad(); critic_loss.backward(); critic_optimizer.step()

# actor updates (decentralized, using centralized critic as baseline)
for i, pi in enumerate(policies):
    sampled_action = pi(obs[i]).rsample()
    all_actions = replace(actions, i, sampled_action)  # replace ith action in joint action vector
    q_val = critic(centralized_state, torch.cat(all_actions, dim=-1))
    actor_loss = -q_val.mean()  # maximize Q through gradient ascent (minimize -Q)
    pi_optimizer[i].zero_grad(); actor_loss.backward(); pi_optimizer[i].step()