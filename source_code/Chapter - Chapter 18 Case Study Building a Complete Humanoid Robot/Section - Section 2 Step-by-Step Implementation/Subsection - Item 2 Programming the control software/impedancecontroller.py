import rclpy
from rclpy.node import Node
# dynamics_lib provides M,C,g,J functions; hardware_api abstracts actuators/sensors
from dynamics_lib import compute_M_C_g, compute_Jacobian
from hardware_api import read_joint_states, write_joint_torques, read_imu

class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller')                 # node name
        self.Kp = [50.0]*NUM_JOINTS
        self.Kd = [2.0]*NUM_JOINTS
        self.max_torque = 40.0
        self.timer = self.create_timer(0.002, self.control_loop) # 500 Hz

    def control_loop(self):
        q, q_dot, ts = read_joint_states()                      # encoder read
        imu = read_imu()                                        # base estimation
        q_des, qd_des, qdd_des = self.get_desired_trajectory()  # from planner
        M, C, g = compute_M_C_g(q, q_dot)                       # model comp
        # PD error
        e = q - q_des
        ed = q_dot - qd_des
        tau_ff = M @ qdd_des + C @ qd_des + g                  # feedforward
        tau_fb = - (self.Kp * e + self.Kd * ed)                # feedback
        tau = tau_ff + tau_fb
        # torque safety clipping
        tau = np.clip(tau, -self.max_torque, self.max_torque)
        write_joint_torques(tau)                                # send to HW

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()