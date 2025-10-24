def estimate_energy(torque, omega, dt, R, k_e, eta_tx=0.95):
    """
    torque, omega: numpy arrays sampled over time (SI units).
    dt: timestep.
    R: winding resistance.
    k_e: motor back-emf constant.
    eta_tx: transmission efficiency.
    Returns estimated electrical energy (J).
    """
    # mechanical power per sample
    P_mech = torque * omega
    # back-emf voltage per sample (approx)
    V_emf = k_e * omega
    # current (approx from torque constant equality)
    I = torque / (k_e)  # assumes k_t = k_e
    P_elec = R * I**2 + V_emf * I
    # account for transmission losses
    P_in = P_elec / eta_tx
    return (P_in * dt).sum()