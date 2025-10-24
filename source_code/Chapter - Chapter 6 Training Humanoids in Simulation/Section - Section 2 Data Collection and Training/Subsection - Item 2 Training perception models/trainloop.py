# model: perception network; domain_clf: domain classifier for w(x)
# synth_loader, real_loader: data loaders; optimizer: for model
for epoch in range(epochs):
    synth_iter = iter(synth_loader)
    real_iter = iter(real_loader)
    for step in range(steps_per_epoch):
        xs, ys = next(synth_iter)                    # synthetic batch
        xr, yr = next(real_iter)                    # real batch (may be small)
        # concatenate and compute domain weights (1 for real, 0 for sim baseline)
        x_all = torch.cat([xs, xr], dim=0)
        # domain logits -> probability real; use to estimate importance weight
        with torch.no_grad():
            d_logits = domain_clf(feature_extractor(x_all))
            p_real = torch.sigmoid(d_logits)        # estimate p_real(x)
            p_sim  = 1.0 - p_real
            w = (p_real / (p_sim + 1e-6)).clamp(max=10.0)  # importance weights
        preds = model(x_all)
        loss_task = task_loss(preds, torch.cat([ys, yr], dim=0))
        loss_weighted = (w * loss_task).mean()      # approximate eq. (1)
        loss_domain = domain_loss(domain_clf(feature_extractor(xs)), 0) \
                    + domain_loss(domain_clf(feature_extractor(xr)), 1)
        loss = loss_weighted + lambda_dom*loss_domain + lambda_reg*reg_loss(model)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        # scheduler step and periodic evaluation on real validation set