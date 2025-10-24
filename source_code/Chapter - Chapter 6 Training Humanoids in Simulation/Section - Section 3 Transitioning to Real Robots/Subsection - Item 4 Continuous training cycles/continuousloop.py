def collect_real(robot, duration):                      # record sensors + commands
    return robot.record(duration)

def estimate_discrepancy(real_data, sim_data):         # compute embedding KL/MMD
    # placeholder: return scalar divergence
    return compute_divergence(real_data.emb, sim_data.emb)

def update_sim_params(sim, real_stats):                 # adjust domain randomization
    sim.params.adapt(real_stats)                        # e.g., friction, latency ranges

def fine_tune(model, dataset, lr=1e-4, steps=1000):     # incremental fine-tune
    for _ in range(steps):
        batch = dataset.sample()
        loss = model.train_step(batch)                  # supervised or RL update
    return model

def safety_validate(model, shadow_logs, thresholds):    # run shadow tests
    metrics = evaluate_shadow(model, shadow_logs)
    return metrics.pass_all(thresholds)

# main loop
while True:
    real_batch = collect_real(robot, duration=60)       # one-minute captures
    sim_batch = sim.generate(seed=real_batch.seed)      # simulate matched episodes
    div = estimate_discrepancy(real_batch, sim_batch)
    if div > DIVERGENCE_THRESHOLD:
        update_sim_params(sim, real_batch.stats)
    dataset.add(real=real_batch, sim=sim_batch)         # mixed replay buffer
    model = fine_tune(model, dataset)                   # update offline or online
    if safety_validate(model, shadow_logs, thresholds):
        deploy(model)                                   # staged production rollout