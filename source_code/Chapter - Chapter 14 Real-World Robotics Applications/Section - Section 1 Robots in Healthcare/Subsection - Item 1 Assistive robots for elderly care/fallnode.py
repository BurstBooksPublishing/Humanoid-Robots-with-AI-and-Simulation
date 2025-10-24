#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Bool
# simplified: subscribe to IMU and skeleton detection topics, publish assist trigger

class FallDetector(Node):
    def __init__(self):
        super().__init__('fall_detector')
        self.create_subscription(Imu, '/wearable/imu', self.imu_cb, 10)
        self.create_subscription(Bool, '/vision/fall_flag', self.vision_cb, 10)
        self.trigger_pub = self.create_publisher(Bool, '/assist/trigger', 10)
        self.imu_fall = False
        self.vision_fall = False

    def imu_cb(self, msg):
        acc = (msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 +
               msg.linear_acceleration.z**2)**0.5
        self.imu_fall = acc > 25.0  # # g threshold, tuned empirically

    def vision_cb(self, msg):
        self.vision_fall = msg.data  # True if vision pipeline flags fall

    def timer_cb(self):
        # fused decision: require both signals or strong vision confidence
        if (self.imu_fall and self.vision_fall) or self.vision_fall:
            self.trigger_pub.publish(Bool(data=True))  # initiate assist behavior

def main(args=None):
    rclpy.init(args=args)
    node = FallDetector()
    node.create_timer(0.5, node.timer_cb)  # check twice per second
    rclpy.spin(node)
    rclpy.shutdown()