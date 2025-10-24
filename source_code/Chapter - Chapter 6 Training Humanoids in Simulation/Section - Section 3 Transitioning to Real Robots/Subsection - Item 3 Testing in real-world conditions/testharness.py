#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
# minimal imports; assume controllers publish joint states and cmd topics

class TestHarness(Node):
    def __init__(self):
        super().__init__('test_harness')
        self.emergency_sub = self.create_subscription(Bool, '/safety/emergency', self._emergency_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self._imu_cb, 100)
        # open local log file for synchronized telemetry
        self.log = open('/var/log/humanoid_tests/session.log','a')  # // comment: ensure permissions
        self.emergency = False

    def _emergency_cb(self, msg):
        self.emergency = msg.data
        if self.emergency:
            self.get_logger().warn('Emergency stop triggered')  # // comment: alert operator

    def _imu_cb(self, msg):
        if self.emergency:
            return
        # log timestamped IMU data; minimal processing here
        self.log.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec},"
                       f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n")

    def run_trial(self, task_fn, timeout_s=30.0):
        start = self.get_clock().now()
        task_fn()  # // comment: publishes commands to controllers
        while (self.get_clock().now() - start).nanoseconds/1e9 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.emergency:
                self.get_logger().error('Abort: emergency stop')
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = TestHarness()
    # task_fn should be provided: short sample below omitted for clarity
    rclpy.shutdown()