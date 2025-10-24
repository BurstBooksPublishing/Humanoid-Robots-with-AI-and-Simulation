import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# Simple running average estimator for engagement
class EngagementNode(Node):
    def __init__(self):
        super().__init__('engagement_node')
        self.pub = self.create_publisher(Float32, 'engagement_score', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)  # 2 Hz
        self.window = []  # sliding window of recent scores
    def timer_cb(self):
        visual = self.read_visual_attention()  # placeholder sensor fusion
        audio = self.read_audio_activity()
        prox = self.read_proximity()
        # weighted fusion, ensure output in [0,1]
        score = max(0.0, min(1.0, 0.6*visual + 0.3*audio + 0.1*(1-prox)))
        self.window.append(score)
        if len(self.window) > 10:
            self.window.pop(0)
        avg = sum(self.window)/len(self.window)
        msg = Float32()
        msg.data = float(avg)
        self.pub.publish(msg)
    def read_visual_attention(self): return 0.8  # stub: replace with detector
    def read_audio_activity(self): return 0.2
    def read_proximity(self): return 0.1
def main(args=None):
    rclpy.init(args=args); node = EngagementNode(); rclpy.spin(node)