def compute_zmp(contact_forces, contact_points, contact_torques):
    """
    contact_forces: list of (fx,fy,fz) in Newtons  # measured contact forces
    contact_points: list of (x,y,z) in meters       # contact positions
    contact_torques: list of (tx,ty,tz) in N*m      # local contact moments
    returns: (px, py) planar ZMP in meters
    """
    num = [0.0, 0.0]      # numerator for x and y
    denom = 0.0          # sum of vertical forces
    for f, r, tau in zip(contact_forces, contact_points, contact_torques):
        fx, fy, fz = f
        rx, ry, rz = r
        tx, ty, tz = tau
        denom += fz
        # moment contributions: (r x f)_z + tau_z
        mz = rx * fy - ry * fx + tz
        # planar ZMP components (assumes forces act at ground height)
        num[0] += -mz * 0.0 + ry * fz  # px numerator -> simplified sign convention
        num[1] +=  mz * 0.0 - rx * fz  # py numerator -> simplified sign conv
    if abs(denom) < 1e-6:
        return None  # no contact, handle as fall condition
    px = num[0] / denom
    py = num[1] / denom
    return px, py
# Note: Real implementation must handle coordinate frames and sign conventions.