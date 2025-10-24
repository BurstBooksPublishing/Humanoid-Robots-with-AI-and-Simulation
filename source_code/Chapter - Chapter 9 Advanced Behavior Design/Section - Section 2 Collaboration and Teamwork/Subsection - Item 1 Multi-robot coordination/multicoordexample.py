import rclpy
from rclpy.node import Node
# message types are simplified placeholders
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

ALPHA = 0.1  # consensus gain (tune for network)
PUBLISH_RATE = 10.0  # Hz

class FormationConsensus(Node):
    def __init__(self, robot_id):
        super().__init__('formation_consensus_'+str(robot_id))
        self.robot_id = robot_id
        self.offset = 0.0  # scalar offset along formation axis
        self.neighbor_offsets = {}  # map neighbor_id -> offset
        self.pub = self.create_publisher(Float32, '/desired_offset', 10)  # publish local offset
        # subscribe to robot_state topic to extract neighbors' offsets
        self.create_subscription(PoseStamped, '/robot_state', self.state_cb, 50)
        self.create_timer(1.0 / PUBLISH_RATE, self.loop_cb)

    def state_cb(self, msg):
        # parse neighbor id and offset encoded in msg.header.frame_id for brevity
        neighbor_id = int(msg.header.frame_id)
        neighbor_offset = msg.pose.position.x  # offset encoded in pose.x
        self.neighbor_offsets[neighbor_id] = neighbor_offset  # update neighbor info

    def loop_cb(self):
        # discrete consensus update using eq.(\ref{eq:consensus})
        sum_diff = 0.0
        for v in self.neighbor_offsets.values():
            sum_diff += (v - self.offset)
        if len(self.neighbor_offsets) > 0:
            self.offset += ALPHA * sum_diff
        out = Float32()
        out.data = float(self.offset)
        self.pub.publish(out)  # broadcast updated offset
        # minimal logging for debugging
        self.get_logger().debug(f'robot {self.robot_id} offset {self.offset:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = FormationConsensus(robot_id=1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()