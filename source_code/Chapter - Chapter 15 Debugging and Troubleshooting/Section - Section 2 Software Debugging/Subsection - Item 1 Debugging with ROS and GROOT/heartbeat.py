import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import time

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat') 
        self.pub = self.create_publisher(Header, 'heartbeat', 10)  # timestamped header
        self.sub = self.create_subscription(Header, 'heartbeat_echo', self.on_echo, 10)
        self.timer = self.create_timer(0.01, self.send)  # 100 Hz heartbeat
    def send(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()  # publisher timestamps
        msg.frame_id = 'heartbeat'  # small metadata
        self.pub.publish(msg)
    def on_echo(self, msg):
        # compute latency: difference between now and original stamp
        now = self.get_clock().now().nanoseconds
        sent = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        latency_ms = (now - sent) / 1e6
        self.get_logger().info(f'heartbeat RTT {latency_ms:.3f} ms')  # short inline comment
def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()