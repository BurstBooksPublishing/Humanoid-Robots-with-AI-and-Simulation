import numpy as np

# pairs: list of (t_ref, t_local) in seconds; collected with hardware timestamps
pairs = np.loadtxt('timestamp_pairs.csv', delimiter=',')  # t_ref,t_local

t_ref = pairs[:,0]
t_loc = pairs[:,1]

# linear fit t_loc = alpha * t_ref + beta
A = np.vstack([t_ref, np.ones_like(t_ref)]).T
alpha, beta = np.linalg.lstsq(A, t_loc, rcond=None)[0]

residuals = t_loc - (alpha * t_ref + beta)
jitter_std = np.std(residuals)

print(f"alpha(skew) = {alpha:.9f}, beta(offset) = {beta:.6f}s")
print(f"residual jitter std = {jitter_std*1e3:.3f} ms")

# optional: detect drift trend (skew change) over time windows
# split into windows to see changing alpha
def windowed_skew(t_ref, t_loc, w=100):
    skews=[]
    for i in range(0, len(t_ref)-w, w):
        A = np.vstack([t_ref[i:i+w], np.ones(w)]).T
        a,_ = np.linalg.lstsq(A, t_loc[i:i+w], rcond=None)[0]
        skews.append(a)
    return np.array(skews)

skews = windowed_skew(t_ref, t_loc)
print("skew variation (ppm):", (skews - alpha).mean()*1e6)