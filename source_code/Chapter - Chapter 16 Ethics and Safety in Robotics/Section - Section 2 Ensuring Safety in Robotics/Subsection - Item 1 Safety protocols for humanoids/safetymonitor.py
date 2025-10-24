#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.create_subscription(Float32, 'proximity/fused_range', self.range_cb, 10)  # m
        self.create_subscription(Float32, 'base/velocity', self.vel_cb, 10)           # m/s
        self.pub_cmd = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.pub_stop = self.create_publisher(Bool, 'hard_stop', 10)
        self.v = 0.0
        self.r = 10.0
        self.a_brake = 1.5  # guaranteed decel m/s^2
        self.d_min = 0.15
        self.delta_lat = 0.05

    def vel_cb(self, msg):
        self.v = msg.data

    def range_cb(self, msg):
        self.r = msg.data
        d_safe = self.d_min + (self.v*self.v)/(2.0*self.a_brake) + self.delta_lat
        if self.r <= self.d_min:
            # immediate hard stop
            self.pub_stop.publish(Bool(data=True))
        elif self.r <= d_safe:
            # publish scaled velocity command (soft stop)
            t = max(0.0, (self.r - self.d_min) / max(1e-3, d_safe - self.d_min))
            cmd = Twist()
            cmd.linear.x = self.v * t  # scale down linearly
            self.pub_cmd.publish(cmd)
        else:
            # safe to continue; publish pass-through command (not shown)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()