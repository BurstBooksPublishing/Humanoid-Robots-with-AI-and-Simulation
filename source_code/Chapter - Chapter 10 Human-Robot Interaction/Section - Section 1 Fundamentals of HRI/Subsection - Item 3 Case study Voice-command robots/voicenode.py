# ROS2-like pseudocode; minimal error handling shown
import rclpy
from rclpy.node import Node

class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__('voice_cmd')                     # node name
        self.sub_audio = self.create_subscription(AudioMsg, '/mics/beamed', self.on_audio, 1)
        self.pub_cmd = self.create_publisher(CmdMsg, '/robot/cmd', 1)
        self.intent_model = load_intent_model()           # on-device intent classifier
        self.safety = SafetyChecker()                     # checks kinematic, env constraints

    def on_audio(self, audio):
        if not vad_detected(audio):                       # on-device VAD
            return
        u, conf = on_device_asr(audio)                    # fast on-device ASR
        if conf < 0.6:                                    # low confidence fallback
            u, conf = edge_asr_offload(audio)            # encrypted edge call
        a = self.intent_model.predict(u)                  # intent probs
        action, prob = argmax_with_confidence(a)          # choose best intent
        if prob < 0.7 or not self.safety.validate(action):
            self.request_confirmation(action)            # ask user or abort
            return
        self.pub_cmd.publish(translate_to_cmd(action))    # safe command issued

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()