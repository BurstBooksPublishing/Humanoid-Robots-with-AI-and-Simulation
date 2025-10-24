# Example: minimal material ranking; integrate with full material DB in production.
materials = [
    {"name":"Al7075", "mass":0.8, "GWP":8.1, "recyclability":0.9, "crit":0.2, "tox":0.1},
    {"name":"CF_recycled","mass":0.5,"GWP":45.0,"recyclability":0.4,"crit":0.6,"tox":0.2},
    {"name":"Stainless304","mass":1.2,"GWP":6.0,"recyclability":0.85,"crit":0.3,"tox":0.15},
]
# weighting factors reflect program priorities (sustainability prioritized here)
w = {"alpha":1.0, "beta":1.5, "gamma":2.0, "delta":1.0, "eta":1.0}
ref = {"mass":1.0, "GWP":1.0}
def score(mat):
    m, G, R, C, T = mat["mass"], mat["GWP"], mat["recyclability"], mat["crit"], mat["tox"]
    J = (w["alpha"]*(m/ref["mass"]) + w["beta"]*(G*m/(ref["GWP"]*ref["mass"]))
         - w["gamma"]*R + w["delta"]*C + w["eta"]*T)
    return J
ranked = sorted(materials, key=lambda x: score(x))
for m in ranked:
    print(m["name"], "score=", round(score(m),3))  # lower score preferred