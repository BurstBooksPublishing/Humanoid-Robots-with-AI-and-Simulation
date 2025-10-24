import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState  # sensor input
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # command

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')  # node name
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # subscribe to sensor data
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos)
        # publish trajectory commands to controller
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/command', qos)
        self.current_positions = []
        self.timer = self.create_timer(0.02, self.publish_command)  # 50 Hz control

    def joint_cb(self, msg: JointState):
        self.current_positions = list(msg.position)  # store latest positions

    def publish_command(self):
        if not self.current_positions:
            return
        traj = JointTrajectory()
        traj.joint_names = ['hip', 'knee', 'ankle']  # adjust for actual robot
        point = JointTrajectoryPoint()
        # build a simple small-step trajectory toward a target
        target = [p + 0.01 for p in self.current_positions]  # small positive step
        point.positions = target
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.02 * 1e9)  # 20 ms to next point
        traj.points = [point]
        self.pub.publish(traj)  # publish command

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()