import math

def compute_timing(m_eff, k_eff, f_ctrl, safety_factor=3.0):
    # compute explicit integrator critical dt and choose physics dt
    dt_crit = 2.0 * math.sqrt(m_eff / k_eff)          # eq. (1)
    dt_phys = dt_crit / safety_factor                # safety margin
    f_phys = 1.0 / dt_phys
    n_sub = max(1, math.ceil(f_phys / f_ctrl))       # integer substeps
    return dt_phys, n_sub, f_phys

def apply_to_simulator(sim_config, dt_phys, n_sub):
    # map to Isaac Sim or engine-specific settings (pseudo-API)
    sim_config['physics_step'] = dt_phys              # e.g., set physics timestep
    sim_config['substeps'] = n_sub                    # engine-specific field
    sim_config['solver_iterations'] = 10              # increase for humanoids
    # set contact parameters
    sim_config['friction_coefficient'] = 0.6
    sim_config['restitution'] = 0.05
    # call into simulator init here (implementation-specific)
    return sim_config

# Example usage
dt_phys, n_sub, f_phys = compute_timing(m_eff=2.0, k_eff=1e4, f_ctrl=200.0)
sim_cfg = {}
sim_cfg = apply_to_simulator(sim_cfg, dt_phys, n_sub)
# pass sim_cfg into simulator initialization (engine API call)