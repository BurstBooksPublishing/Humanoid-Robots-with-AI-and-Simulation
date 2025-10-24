import torch, torch.nn as nn
# model: simple LSTM regressor
class LSTMReg(nn.Module):
    def __init__(self, feat_dim, hid=64, layers=2):
        super().__init__()
        self.lstm = nn.LSTM(feat_dim, hid, layers, batch_first=True)
        self.fc = nn.Linear(hid, 1)
    def forward(self, x):
        _, (h, _) = self.lstm(x)            # x: (B, T, F)
        out = self.fc(h[-1])                # last layer hidden state
        return out.squeeze(-1)

model = LSTMReg(feat_dim=32).to('cuda')
opt = torch.optim.Adam(model.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

for epoch in range(50):
    for X_batch, y_batch in train_loader:         # X.shape=(B,T,F)
        X_batch, y_batch = X_batch.cuda(), y_batch.cuda()
        y_hat = model(X_batch)                     # predict RUL
        loss = loss_fn(y_hat, y_batch)
        opt.zero_grad(); loss.backward(); opt.step()
    # simple validation and logging