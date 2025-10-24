#!/usr/bin/env python3
# simple detector: residual-based check and publish fallback mode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
import numpy as np

class FaultDetector(Node):
    def __init__(self):
        super().__init__('fault_detector')
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.cb_js, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.cb_imu, 10)
        self.pub_mode = self.create_publisher(String, '/safety_mode', 10)
        self.x_hat = np.zeros(6)  # state estimate (pos, vel) for key joints
        self.P = np.eye(6) * 1e-3  # covariance
        self.threshold = 16.0  # chi-square threshold for 6 DOF
    def cb_js(self, msg):
        y = np.array(msg.position[:6])
        # simple residual with identity observation
        r = y - self.x_hat[:6]
        s = r.T @ np.linalg.inv(self.P[:6,:6]) @ r
        if s > self.threshold:
            m = String()
            m.data = 'DEGRADED'  # request graceful degradation
            self.pub_mode.publish(m)
        # update simple estimator (low-pass)
        self.x_hat[:6] = 0.9*self.x_hat[:6] + 0.1*y
    def cb_imu(self, msg):
        # detect extreme tilt as potential balance failure -> safe-hold
        pitch = 2*(msg.orientation.x*msg.orientation.y + msg.orientation.w*msg.orientation.z)
        if abs(pitch) > 0.5:
            m = String(); m.data = 'SAFE_HOLD'
            self.pub_mode.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = FaultDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()