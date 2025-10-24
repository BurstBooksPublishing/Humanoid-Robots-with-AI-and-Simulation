# minimal risk estimator for humanoid runtime (ROS/ROS2 integration expected)
THRESHOLD = 0.7  # risk threshold to lower autonomy
AUTONOMY_HIGH = 1.0
AUTONOMY_LOW  = 0.2

def expected_harm(confidence, proximity):
    # simple model: higher confidence reduces misperception risk
    # smaller proximity increases physical harm likelihood
    P_physical = max(0.0, 1.0 - confidence) * (1.0 / (proximity + 0.1))
    H_physical = 10.0  # severity weight (engineer-defined)
    return P_physical * H_physical

def runtime_loop():
    autonomy = AUTONOMY_HIGH
    while True:
        c = get_sensor_confidence()    # perception module returns [0,1]
        p = get_proximity()            # distance in meters to nearest human
        risk = expected_harm(c, p)
        if risk > THRESHOLD:
            autonomy = AUTONOMY_LOW    # throttle autonomy for safety
            engage_safe_behaviors()    # switch to conservative behavior tree
        else:
            autonomy = AUTONOMY_HIGH
        set_autonomy_level(autonomy)   # apply to planner/control stack
        log_risk(risk)                 # secure logging for audit