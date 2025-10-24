import numpy as np
# y_true, y_pred, groups are numpy arrays aligned by index
# groups hold integer group ids; 0..G-1
def per_group_rates(y_true, y_pred, groups):
    G = int(groups.max()) + 1
    tpr = np.zeros(G); fpr = np.zeros(G); counts = np.zeros(G)
    for g in range(G):
        mask = (groups == g)
        counts[g] = mask.sum()
        if counts[g] == 0: continue
        tp = np.logical_and(y_pred==1, y_true==1)[mask].sum()
        fn = np.logical_and(y_pred==0, y_true==1)[mask].sum()
        fp = np.logical_and(y_pred==1, y_true==0)[mask].sum()
        tn = np.logical_and(y_pred==0, y_true==0)[mask].sum()
        tpr[g] = tp / (tp + fn) if (tp + fn)>0 else np.nan
        fpr[g] = fp / (fp + tn) if (fp + tn)>0 else np.nan
    return tpr, fpr, counts
# Example usage (simulate arrays) -> compute disparity metrics