import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class ImuTorqueController(Node):
    def __init__(self):
        super().__init__('imu_torque_controller')
        # subscribe to IMU; do not block heavy computation in callback
        self.create_subscription(Imu, 'imu/data', self.imu_cb, 10)
        # publish joint torques (controller-level topic)
        self.torque_pub = self.create_publisher(Float64MultiArray, 'joint_torques', 10)
        self.dt = 0.002  # expected control loop ~500 Hz
        self.alpha = 0.98
        self.pitch = 0.0

    def imu_cb(self, msg: Imu):
        # extract gyro z (body yaw) and acc pitch estimate (simple example)
        gz = msg.angular_velocity.y  # small-angle mapping; adapt to frame
        acc = msg.linear_acceleration
        pitch_acc = np.arctan2(-acc.x, np.sqrt(acc.y**2 + acc.z**2))
        # complementary update (discrete)
        self.pitch = self.alpha * (self.pitch + gz * self.dt) + (1.0 - self.alpha) * pitch_acc
        # compute a simple balancing torque profile across hip joints
        torque_cmd = Float64MultiArray()
        # two hip actuators: proportional to pitch angle (low-level PD should run elsewhere)
        kp = 30.0
        torque_cmd.data = [ -kp * self.pitch, -kp * self.pitch ]  # left/right hip
        # safety clamp
        torque_cmd.data = [max(min(t, 50.0), -50.0) for t in torque_cmd.data]
        self.torque_pub.publish(torque_cmd)  # publish to actuator controller

def main(args=None):
    rclpy.init(args=args)
    node = ImuTorqueController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Note: Replace with realtime-safe patterns and ros2_control for production.