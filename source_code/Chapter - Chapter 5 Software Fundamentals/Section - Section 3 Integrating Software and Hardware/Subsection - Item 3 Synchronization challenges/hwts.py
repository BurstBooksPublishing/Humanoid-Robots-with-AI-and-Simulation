import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        # subscribe to IMU; assume hardware timestamp in header.stamp
        self.sub = self.create_subscription(Imu, 'imu/data_raw', self.cb, 10)

    def cb(self, msg: Imu):
        # prefer hardware timestamp if present (ROS Time type)
        hw_ts = msg.header.stamp  # rclpy.Time from hardware
        # convert to seconds (float) safely
        ts_sec = hw_ts.sec + hw_ts.nanosec * 1e-9
        recv_time = time.time()   # local wall-clock receive time
        latency = recv_time - ts_sec
        # log latency and raise alarms if latency exceeds threshold
        if latency > 0.005:  # 5 ms threshold for joint-level control
            self.get_logger().warn(f'High sensor latency: {latency:.6f} s')
        # forward corrected timestamp into estimator/controller here

def main():
    rclpy.init()
    node = LatencyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()