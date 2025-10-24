import rclpy
from rclpy.node import Node
import numpy as np
import csv
# Subscribe to a stamped sensor topic and measure arrival latency.
class LatencyLogger(Node):
    def __init__(self):
        super().__init__('latency_logger')
        self.sub = self.create_subscription(
            SensorMsgType,  # replace with actual message type
            '/camera/image_stamped',
            self.cb, 10)
        self.latencies = []

    def cb(self, msg):
        # compute latency using message header stamp and current time
        now = self.get_clock().now().to_msg()  # rclpy time msg
        send_ts = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        recv_ts = now.sec + now.nanosec*1e-9
        lat = recv_ts - send_ts
        self.latencies.append(lat)
        if len(self.latencies) % 1000 == 0:
            self._write_stats()

    def _write_stats(self):
        a = np.array(self.latencies)
        p95 = np.percentile(a,95)
        p99 = np.percentile(a,99)
        mean = a.mean()
        # append summary to CSV (simple I/O acceptable in analysis mode)
        with open('latency_stats.csv','a',newline='') as f:
            csv.writer(f).writerow([len(a), mean, p95, p99])

def main(args=None):
    rclpy.init(args=args)
    node = LatencyLogger()
    rclpy.spin(node)
    rclpy.shutdown()
# Note: SensorMsgType must contain header.stamp for accurate measurement.