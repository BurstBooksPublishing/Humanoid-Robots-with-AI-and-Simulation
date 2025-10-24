import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time, statistics

class CommProbe(Node):
    def __init__(self, topic='/joint_states', expected_rate=100.0):
        super().__init__('comm_probe')
        self.sub = self.create_subscription(JointState, topic, self.cb, 10)
        self.expected_dt = 1.0/expected_rate
        self.arrivals = []  # store interarrival times
        self.latencies = [] # store latency samples
        self.last_recv = None
        self.miss_count = 0
        self.recv_count = 0

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        pub_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        latency = now - pub_stamp  # approximate one-way latency
        self.latencies.append(latency)
        if self.last_recv is not None:
            dt = now - self.last_recv
            self.arrivals.append(dt)
            # crude loss detection: gap larger than 1.5x expected dt
            if dt > 1.5 * self.expected_dt:
                self.miss_count += int(round(dt / self.expected_dt)) - 1
        self.last_recv = now
        self.recv_count += 1
        # periodic summary
        if self.recv_count % 500 == 0:
            print(f"recv={self.recv_count} miss~={self.miss_count} "
                  f"latency_mean={statistics.mean(self.latencies):.3f}s "
                  f"jitter={statistics.pstdev(self.arrivals):.3f}s")

def main(args=None):
    rclpy.init(args=args)
    probe = CommProbe()
    rclpy.spin(probe)
    probe.destroy_node()
    rclpy.shutdown()
# publisher must stamp messages accurately for latency to be meaningful