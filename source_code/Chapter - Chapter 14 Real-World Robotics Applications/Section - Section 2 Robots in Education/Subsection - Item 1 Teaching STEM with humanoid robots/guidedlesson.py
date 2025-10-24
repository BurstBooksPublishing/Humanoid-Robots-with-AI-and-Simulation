#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Simple safety thresholds (example values)
MAX_JOINT_VEL = 1.0  # rad/s
MAX_TORQUE = 2.0     # Nm

class GuidedLessonBridge(Node):
    def __init__(self):
        super().__init__('guided_lesson_bridge')
        # student-level commands (block/program) -> check -> publish to controller
        self.sub_cmd = self.create_subscription(String, '/student_command', self.cb_cmd, 10)
        self.pub_ctrl = self.create_publisher(String, '/low_level_command', 10)
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        self.timer = self.create_timer(1.0, self.heartbeat)
        self.last_joint_state = None

    def cb_cmd(self, msg):
        # Basic parsing and safety check (placeholder)
        cmd = msg.data
        if "move" in cmd:
            # enforce safe velocity and confirm
            safe_cmd = f"{cmd};max_vel={MAX_JOINT_VEL}"
            self.pub_ctrl.publish(String(data=safe_cmd))
        else:
            self.pub_ctrl.publish(msg)  # pass-through for benign commands

    def cb_joint(self, msg):
        self.last_joint_state = msg
        # simple current/torque check (real system uses real readings)
        # inline comment: abort if torques exceed safe thresholds (pseudo)
        # if max(msg.effort) > MAX_TORQUE: publish emergency stop

    def heartbeat(self):
        # periodic health report (teacher dashboard can subscribe)
        self.get_logger().info('guided lesson active')

def main(args=None):
    rclpy.init(args=args)
    node = GuidedLessonBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()