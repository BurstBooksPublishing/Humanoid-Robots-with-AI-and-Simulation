import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import Float64
import math

class CapturePointNode(Node):
    def __init__(self):
        super().__init__('capture_point_node')        # node name
        self.zc = 0.9                                # COM nominal height (m)
        self.g = 9.81
        self.pub = self.create_publisher(Point,'/capture_point',10)
        self.create_subscription(TwistStamped,'/com_state',self.cb_state,10)
    def cb_state(self,msg):
        # msg.twist.linear.x = com velocity; msg.twist.linear.y not used here
        v = msg.twist.linear.x
        x = msg.twist.linear.z                        # using z field for COM x (example)
        omega = math.sqrt(self.g/self.zc)             # from Equation (1)
        x_cp = x + v/omega                            # Equation (2)
        pt = Point()
        pt.x = x_cp; pt.y = 0.0; pt.z = 0.0
        self.pub.publish(pt)                          # publish capture point

def main(args=None):
    rclpy.init(args=args)
    node = CapturePointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()