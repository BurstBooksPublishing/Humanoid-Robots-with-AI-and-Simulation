import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        qos = rclpy.qos.QoSProfile(depth=10)    # bounded queue
        self.imu_sub = Subscriber(self, Imu, '/imu', qos_profile=qos)
        self.joint_sub = Subscriber(self, JointState, '/joint_states', qos_profile=qos)
        # sync with small slack to avoid mismatched timestamps
        self.sync = ApproximateTimeSynchronizer([self.imu_sub, self.joint_sub], queue_size=20, slop=0.01)
        self.sync.registerCallback(self.fused_callback)
        self.max_staleness = rclpy.duration.Duration(seconds=0.05) # drop >50ms stale

    def fused_callback(self, imu, joints):
        now = self.get_clock().now()
        # drop stale messages to avoid control using old data
        if (now - rclpy.time.Time.from_msg(imu.header.stamp)) > self.max_staleness:
            return  # skip stale imu
        # compute fused state (real implementation omitted)
        # publish or feed controller