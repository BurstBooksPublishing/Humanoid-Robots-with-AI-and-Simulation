# Simple ranking of structural materials for humanoid components
materials = {  # density (kg/m3), E (GPa), yield (MPa)
    "Steel_1045": (7850, 210.0, 530.0),
    "Al_7075":   (2810, 71.7, 503.0),
    "Ti6Al4V":   (4430, 114.0, 880.0),
    "CFRP":      (1600, 120.0, 900.0)  # anisotropic; approximate along fiber
}
# compute specific metrics and sort
scores = {}
for name, (rho, E, sigma_y) in materials.items():
    spec_stiff = E*1e9 / rho          # Pa/(kg/m3) -> m2/s2
    spec_strength = sigma_y*1e6 / rho # Pa/(kg/m3)
    scores[name] = (spec_stiff, spec_strength)
# print ranking by specific stiffness
for name, vals in sorted(scores.items(), key=lambda kv: kv[1][0], reverse=True):
    print(f"{name}: specific_stiff={vals[0]:.2e}, specific_strength={vals[1]:.2e}")