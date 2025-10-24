import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

class GrootBridge(Node):
    def __init__(self):
        super().__init__('groot_bridge')
        # perception updates blackboard variable 'object_visible'
        self.create_subscription(Bool, 'perception/object_visible', self.percept_cb, 10)
        # blackboard published as simple JSON string (could be a proper service)
        self.bb_pub = self.create_publisher(String, 'blackboard', 10)
        # action command topic for low-level grasp controller
        self.grasp_pub = self.create_publisher(PoseStamped, 'cmd/grasp_pose', 10)
        # action status feedback
        self.create_subscription(String, 'skill/grasp/status', self.grasp_status_cb, 10)
        self.blackboard = {'object_visible': False}
    def percept_cb(self, msg):
        self.blackboard['object_visible'] = bool(msg.data)
        self._publish_blackboard()                       # update tree runtime
    def _publish_blackboard(self):
        js = str(self.blackboard)                        # simple serialization
        m = String(data=js)
        self.bb_pub.publish(m)
    def grasp_status_cb(self, msg):
        # forward status to BT runtime (not shown): 'SUCCESS','FAIL','RUNNING'
        pass
    def request_grasp(self, pose: PoseStamped):
        self.grasp_pub.publish(pose)                     # trigger low-level controller
        # runtime listens to status topic for completion
def main(args=None):
    rclpy.init(args=args)
    node = GrootBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
# run this node alongside the BT runtime and the exported GROOT tree.