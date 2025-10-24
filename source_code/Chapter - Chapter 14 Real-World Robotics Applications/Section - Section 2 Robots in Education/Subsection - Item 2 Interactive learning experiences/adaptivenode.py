import rclpy
from rclpy.node import Node

# topics: '/engagement' (Float32), '/success' (Float32), '/behavior_cmd' (String)
class AdaptiveLessonNode(Node):
    def __init__(self):
        super().__init__('adaptive_lesson')
        self.d = 0.5                    # initial difficulty
        self.alpha = 0.3
        self.beta = 0.2
        self.target_success = 0.8
        self.target_engagement = 0.6
        self.create_subscription(Float32, '/engagement', self.eng_cb, 10)
        self.create_subscription(Float32, '/success', self.succ_cb, 10)
        self.pub = self.create_publisher(String, '/behavior_cmd', 10)
        self.last_eng = 0.0
        self.last_succ = 0.0

    def eng_cb(self, msg):
        self.last_eng = msg.data
        self.update_and_issue()

    def succ_cb(self, msg):
        self.last_succ = msg.data
        self.update_and_issue()

    def update_and_issue(self):
        # update difficulty per eq.(2)
        delta = self.alpha*(self.last_succ - self.target_success)
        delta += self.beta*(self.last_eng - self.target_engagement)
        self.d = max(0.0, min(1.0, self.d + delta))    # clip
        cmd = self.select_behavior()
        self.pub.publish(String(data=cmd))

    def select_behavior(self):
        # simple policy mapping difficulty to behavior
        if self.last_succ < 0.5:
            return 'hint'      # provide scaffold
        if self.last_eng < 0.4:
            return 'reengage'  # use gesture/smile/audio to regain attention
        if self.d < 0.4:
            return 'practice'
        if self.d < 0.8:
            return 'challenge'
        return 'assessment'

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveLessonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()