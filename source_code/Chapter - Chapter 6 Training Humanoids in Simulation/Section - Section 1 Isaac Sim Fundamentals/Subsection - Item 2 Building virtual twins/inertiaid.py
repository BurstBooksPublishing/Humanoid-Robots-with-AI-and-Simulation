import numpy as np
from scipy.optimize import least_squares

# measured arrays: q, dq, ddq, tau_real (shape: T x n)
# model_torque(params, q, dq, ddq) returns tau_model for all samples
def residuals(params, q, dq, ddq, tau_real):
    tau_model = model_torque(params, q, dq, ddq)  # compute using RBD model
    return (tau_real - tau_model).ravel()

# initial guess from CAD and datasheet
p0 = initial_params_from_cad()
res = least_squares(residuals, p0, args=(q, dq, ddq, tau_real),
                    xtol=1e-8, ftol=1e-8, max_nfev=2000)
p_est = res.x  # estimated inertial and friction parameters
# write p_est back to the simulator's articulation parameters (USD/Omniverse)