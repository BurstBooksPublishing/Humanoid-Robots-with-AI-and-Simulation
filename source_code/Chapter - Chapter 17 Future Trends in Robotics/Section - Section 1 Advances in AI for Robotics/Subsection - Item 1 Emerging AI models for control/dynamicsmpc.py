import torch, torch.nn as nn
# small MLP dynamics model
class DynModel(nn.Module):
    def __init__(self, s_dim, a_dim, h=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(s_dim+a_dim, h), nn.ReLU(),
            nn.Linear(h, h), nn.ReLU(),
            nn.Linear(h, s_dim)  # predict state delta
        )
    def forward(self, s, a):
        inp = torch.cat([s, a], dim=-1)
        return s + self.net(inp)  # predict next state

# training loop (data: states, actions, next_states tensors)
model = DynModel(s_dim=50, a_dim=12).to('cuda')
opt = torch.optim.Adam(model.parameters(), lr=1e-3)
for epoch in range(100):
    pred = model(states, actions)           # batch predict
    loss = ((pred - next_states)**2).mean() # MSE on delta
    opt.zero_grad(); loss.backward(); opt.step()

# simple shooting MPC at runtime
def mpc_shoot(x0, horizon=10, candidates=512):
    # sample random action sequences, simulate with model
    A = torch.randn(candidates, horizon, 12, device='cuda') * 0.5
    X = x0.repeat(candidates,1)
    total_cost = torch.zeros(candidates, device='cuda')
    for t in range(horizon):
        a = A[:,t,:]
        X = model(X, a)                         # rollout on learned model
        total_cost += cost_fn(X, a)            # task-specific cost
    best = torch.argmin(total_cost)
    return A[best,0,:].cpu().numpy()           # return first action