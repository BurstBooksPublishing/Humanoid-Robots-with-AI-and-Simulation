import numpy as np
# simple detector: compute residuals and use moving window stats
WINDOW = 100  # sample window
TH_POS = 0.02  # rad threshold for position residual
TH_CURR = 0.5  # A threshold for current anomaly

def detect_failures(cmd_pos_hist, meas_pos_hist, curr_hist):
    # compute residuals over window
    pos_res = np.array(cmd_pos_hist[-WINDOW:]) - np.array(meas_pos_hist[-WINDOW:])
    curr_mean = np.mean(curr_hist[-WINDOW:])
    pos_std = np.std(pos_res)
    pos_bias = np.mean(pos_res)
    # flag conditions: bias or large variance or excess current
    flags = {}
    flags['backlash_like'] = abs(pos_bias) > TH_POS and pos_std < TH_POS
    flags['friction_like'] = curr_mean > TH_CURR and pos_std < 0.01
    flags['intermittent'] = pos_std > 3*0.01  # high jitter
    return flags

# example usage (called per control loop with history buffers)