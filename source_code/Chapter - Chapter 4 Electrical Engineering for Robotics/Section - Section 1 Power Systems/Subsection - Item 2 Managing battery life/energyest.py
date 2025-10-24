# Simple SOC and runtime estimator (embedded use)
def update_soc(soc, Q_ah, I_a, dt_s, eta_reg=0.9):
    # I_a positive for discharge, negative for regen
    # Coulomb count update
    dAh = I_a * dt_s / 3600.0
    if I_a < 0:  # regen: apply charge efficiency
        dAh *= eta_reg
    soc_new = soc - dAh / Q_ah
    return max(0.0, min(1.0, soc_new))  # clamp

def predict_runtime(soc, Q_ah, V_nom, I_avg_a):
    # Predict runtime (s) under constant average current
    Ah_rem = soc * Q_ah
    if I_avg_a <= 0:
        return float('inf')  # no discharge
    return Ah_rem / I_avg_a * 3600.0

# Example usage
if __name__ == "__main__":
    soc = 0.9
    Q_ah = 10.0  # 10 Ah pack
    V_nom = 25.9
    # update with sample current and dt
    soc = update_soc(soc, Q_ah, I_a=15.0, dt_s=0.5)  # 15 A burst
    runtime_s = predict_runtime(soc, Q_ah, V_nom, I_avg_a=8.0)
    # runtime_s used to trigger higher-efficiency gait if below threshold