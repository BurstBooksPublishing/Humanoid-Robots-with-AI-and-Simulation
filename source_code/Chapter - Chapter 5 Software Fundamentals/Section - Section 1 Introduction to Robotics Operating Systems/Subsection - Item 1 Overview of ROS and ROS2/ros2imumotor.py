import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from std_msgs.msg import String

class ImuMotorBridge(Node):
    def __init__(self):
        super().__init__('imu_motor_bridge')
        # IMU: low-latency, tolerate lossy transport for high rate
        imu_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            deadline=Duration(seconds=0, nanoseconds=10000000)  # 10ms deadline
        )
        # Motor commands: reliable, small depth to preserve freshness
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', imu_qos)
        self.create_subscription(String, '/motor/commands', self.cmd_callback, cmd_qos)
        self.timer = self.create_timer(0.005, self.publish_imu)  # 200 Hz

    def publish_imu(self):
        msg = Imu()
        # fill msg.header and sensor fields (omitted for brevity)
        self.imu_pub.publish(msg)

    def cmd_callback(self, msg):
        # parse and forward commands to actuator controller (RT path)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ImuMotorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()