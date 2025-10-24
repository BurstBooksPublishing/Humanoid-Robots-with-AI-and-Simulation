import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import csv, time

class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.safe_pub = self.create_publisher(Bool, '/safe_stop', 10)
        self.threshold = 0.05  # seconds allowed latency
        # Welford variables
        self.n = 0; self.mean = 0.0; self.M2 = 0.0
        self.csv = open('latency_log.csv','w',newline='')  # persistent log
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['t_now','t_msg','latency_s'])
    def cb(self, msg):
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        t_now = self.get_clock().now().nanoseconds*1e-9
        latency = t_now - t_msg
        # update running mean and variance
        self.n += 1
        delta = latency - self.mean
        self.mean += delta / self.n
        self.M2 += delta * (latency - self.mean)
        self.writer.writerow([t_now, t_msg, latency])
        # trigger safe stop if exceeded
        if latency > self.threshold:
            self.get_logger().warn(f'Latency {latency:.3f}s > threshold.')
            self.safe_pub.publish(Bool(data=True))  # trigger emergency stop
    def get_stats(self):
        if self.n < 2:
            return self.mean, 0.0
        var = self.M2 / (self.n - 1)
        return self.mean, var**0.5

def main(args=None):
    rclpy.init(args=args)
    node = LatencyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.csv.close()
        node.destroy_node()
        rclpy.shutdown()