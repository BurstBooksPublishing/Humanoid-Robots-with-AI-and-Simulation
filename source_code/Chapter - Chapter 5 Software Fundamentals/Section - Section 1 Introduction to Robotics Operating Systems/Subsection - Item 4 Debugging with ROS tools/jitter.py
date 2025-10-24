#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import statistics
# Simple node to log inter-arrival times for /joint_states
class JitterMonitor(Node):
    def __init__(self):
        super().__init__('jitter_monitor')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10)  # subscriber QoS depth 10
        self.prev_ts = None
        self.deltas = []
    def cb(self, msg: JointState):
        now = self.get_clock().now().nanoseconds
        if self.prev_ts:
            dt = (now - self.prev_ts) * 1e-9  # seconds
            self.deltas.append(dt)
            if len(self.deltas) >= 200:
                mean = statistics.mean(self.deltas)
                std = statistics.stdev(self.deltas)
                self.get_logger().info(f'avg {mean:.6f}s std {std:.6f}s')  # brief log
                self.deltas.clear()
        self.prev_ts = now
def main(args=None):
    rclpy.init(args=args)
    node = JitterMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()