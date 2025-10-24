import numpy as np
import pandas as pd
from sklearn.metrics import precision_recall_fscore_support

# load rollouts exported as CSV (one episode per row)
df = pd.read_csv("rollouts.csv")  # each row: return, success, energy, precision, recall

# basic aggregates
success_rate = df["success"].mean()          # fraction succeeded
mean_return = df["return"].mean()            # average cumulative reward
energy_per_task = df["energy"].mean()        # proxy for power/wear

# compute F1 across perception results if scores per-frame are summarized
precision = df["precision"].mean()
recall = df["recall"].mean()
f1 = (2*precision*recall)/(precision+recall+1e-12)  # safe F1

# bootstrap CI for success rate
def bootstrap_ci(x, B=2000, alpha=0.05):
    n = len(x)
    means = np.array([np.mean(np.random.choice(x, n, replace=True)) for _ in range(B)])
    return np.percentile(means, [100*alpha/2, 100*(1-alpha/2)])

ci_low, ci_high = bootstrap_ci(df["success"].values)

print(f"Success: {success_rate:.3f} CI [{ci_low:.3f},{ci_high:.3f}]")
print(f"Return: {mean_return:.2f}, Energy: {energy_per_task:.2f}, F1: {f1:.3f}")