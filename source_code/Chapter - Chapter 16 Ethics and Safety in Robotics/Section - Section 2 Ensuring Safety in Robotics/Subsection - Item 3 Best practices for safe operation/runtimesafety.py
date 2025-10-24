import time
# sensor APIs return floats; in production use validated drivers
def read_human_distance(): 
    return sensors.depth.closest_human()  # distance in meters

def safe_stop(): 
    robot.motion.set_zero_velocity()  # immediate zero-vel command

SAFETY_DISTANCE = 0.8  # meters conservative initial setting
REACT_TIME = 0.05      # seconds loop latency estimate

while True:
    d = read_human_distance()           # read sensor (blocking or non-blocking)
    v = robot.motion.estimate_velocity()# current body speed
    a_max = robot.motion.max_deceleration()
    d_stop = v * REACT_TIME + v*v/(2*a_max)  # Eq. (1)
    if d - d_stop < SAFETY_DISTANCE:    # imminent violation
        safe_stop()                     # execute hardware E-stops if needed
        robot.logger.warn("Safe stop triggered", d=d, v=v)
    time.sleep(0.01)                    # 100 Hz loop