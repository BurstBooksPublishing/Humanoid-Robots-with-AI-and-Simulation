import numpy as np
# f, g: dynamics functions; h, dh_dx: safety function and its gradient
# u_des: desired control (from planner or policy)
def safety_filter(x, u_des):
    # linearize dynamics: compute a = dh_dx @ f(x), b = dh_dx @ g(x)
    a = dh_dx(x).dot(f(x))               # scalar
    b = dh_dx(x).dot(g(x))               # row vector (1 x m)
    alpha = 5.0 * h(x) if h(x) >= 0 else 5.0 * h(x)  # linear alpha(s)=gamma s
    # QP: min ||u - u_des||^2 s.t. b u >= -a - alpha
    # simple analytical projection for one linear constraint (m dims)
    # compute violation
    rhs = -a - alpha
    lhs = b.dot(u_des)
    if lhs >= rhs:
        return u_des                       # safe already
    # project onto half-space {u : b u >= rhs}
    b_norm_sq = (b.dot(b.T) + 1e-9)
    u_corr = u_des + ((rhs - lhs) / b_norm_sq) * b.T
    return u_corr
# (Call safety_filter at each control step; fallback if correction excessive.)