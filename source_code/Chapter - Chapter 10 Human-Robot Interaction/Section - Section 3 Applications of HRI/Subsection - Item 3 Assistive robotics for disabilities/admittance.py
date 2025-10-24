import rclpy
from rclpy.node import Node
# message types (simplified)
from geometry_msgs.msg import Wrench, Twist

class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')
        self.create_subscription(Wrench, '/force_sensor', self.force_cb, 10)
        self.pub = self.create_publisher(Twist, '/ee_velocity', 10)
        # virtual parameters: tune for compliance vs tracking
        self.M = 1.0  # virtual mass (kg)
        self.D = 20.0  # damping
        self.K = 50.0  # stiffness
        self.x = 0.0; self.x_dot = 0.0  # 1D simplified state
        self.timer = self.create_timer(0.01, self.loop)  # 100 Hz
        self.last_force = 0.0

    def force_cb(self, msg: Wrench):
        # read axial force component; in practice fuse multi-axis data
        self.last_force = msg.force.z

    def loop(self):
        # discrete-time admittance integration (semi-implicit Euler)
        f_ext = self.last_force
        x_ddot = (f_ext - self.D*self.x_dot - self.K*(self.x))/self.M
        self.x_dot += x_ddot * 0.01
        self.x += self.x_dot * 0.01
        # bound velocity for safety
        v = max(min(self.x_dot, 0.2), -0.2)
        t = Twist()
        t.linear.z = v  # publish desired end-effector velocity
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()