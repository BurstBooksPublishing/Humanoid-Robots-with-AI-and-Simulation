import rclpy
from rclpy.node import Node
# msgs are placeholders; replace with real message types in integration
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class GaitConsensusNode(Node):
    def __init__(self):
        super().__init__('gait_consensus') 
        self.phase = 0.0  # local phase [0,1)
        self.coM = Point()  # local CoM estimate
        self.neighbors = {}  # id -> (phase, coM)
        self.eps = 0.1  # consensus step
        # publish local state
        self.phase_pub = self.create_publisher(Float32, 'gait_phase', 10)
        self.com_pub = self.create_publisher(Point, 'com_est', 10)
        # subscribe to neighbor states -- topics are examples
        self.create_subscription(Float32, 'gait_phase', self.on_phase, 10)
        self.create_subscription(Point, 'com_est', self.on_com, 10)
        # periodic timer to run consensus and safety check
        self.create_timer(0.05, self.step)  # 20 Hz

    def on_phase(self, msg):
        # parse neighbor id from header in full implementation
        nid = 'unknown'
        self.neighbors.setdefault(nid, (0.0, Point()))
        self.neighbors[nid] = (msg.data, self.neighbors[nid][1])

    def on_com(self, msg):
        nid = 'unknown'
        self.neighbors.setdefault(nid, (0.0, Point()))
        self.neighbors[nid] = (self.neighbors[nid][0], msg)

    def step(self):
        # consensus on phase using Eq. (1)
        if self.neighbors:
            mean_diff = sum((p - self.phase) for p, _ in self.neighbors.values())
            self.phase += self.eps * mean_diff / max(1, len(self.neighbors))
            self.phase = self.phase % 1.0
        # safety override: if any neighbor too close, pause phase progression
        for _, com in self.neighbors.values():
            if self.too_close(com):
                self.get_logger().warn('Safety pause')  # brief comment
                return  # skip publish to hold gait
        # publish updated state
        self.phase_pub.publish(Float32(data=float(self.phase)))
        self.com_pub.publish(self.coM)

    def too_close(self, other_com):
        # simple distance check; replace with full collision model
        dx = self.coM.x - other_com.x
        dy = self.coM.y - other_com.y
        return (dx*dx + dy*dy) < 0.5*0.5  # 0.5 m safety radius

def main(args=None):
    rclpy.init(args=args)
    node = GaitConsensusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()