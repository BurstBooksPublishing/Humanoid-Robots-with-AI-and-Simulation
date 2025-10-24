import time
from math import isfinite
# controller and sensors are hardware-specific APIs (replace with real drivers)
controller = ControllerAPI()  # set torque, read joint states
sensors = SensorAPI()         # read strain gauges, load cells, IMUs

TORQUE_MAX = 50.0  # Nm, test boundary
TORQUE_STEP = 1.0  # Nm per second
STRain_THRESHOLD = 2000e-6  # microstrain limit

def log_sample(torque):
    s = sensors.read_strain()        # vector of strains
    f = sensors.read_force()         # foot load cell
    controller_state = controller.state()
    # write to CSV or database (omitted) -- minimal inline comments
    print(f"{time.time()},{torque},{s[0]:.6e},{f[0]:.3f},{controller_state.pos}")

def run_torque_ramp(joint_id):
    torque = 0.0
    controller.enable_safety_lock()  # ensure safe start
    while torque <= TORQUE_MAX:
        controller.set_torque(joint_id, torque)  # command torque
        time.sleep(0.2)                           # allow settling
        log_sample(torque)
        if max(abs(x) for x in sensors.read_strain()) > STRain_THRESHOLD:
            controller.set_torque(joint_id, 0.0) # emergency stop
            raise RuntimeError("Strain threshold exceeded")
        torque += TORQUE_STEP
    controller.set_torque(joint_id, 0.0)
    controller.disable_safety_lock()

# Example usage
if __name__ == "__main__":
    run_torque_ramp(joint_id=2)