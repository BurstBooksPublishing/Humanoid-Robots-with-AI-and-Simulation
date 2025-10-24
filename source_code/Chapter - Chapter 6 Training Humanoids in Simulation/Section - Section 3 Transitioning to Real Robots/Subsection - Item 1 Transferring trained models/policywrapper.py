import rclpy  # ROS2 client library
from rclpy.node import Node
import torch  # load policy
# minimal imports for reading joint states and publishing commands
# assume existence of safe_controller() fallback

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        self.policy = torch.jit.load('policy.pt')  # loaded policy
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)  # subscribe
        self.cmd_pub = self.create_publisher(JointTrajectory, '/joint_commands', 10)
        self.safety_limits = {'torque_max': 50.0}  # Nm, example
        self.last_state = None

    def js_cb(self, msg):
        self.last_state = msg
        action = self.infer_action(msg)
        if self.check_safety(action):
            self.cmd_pub.publish(self.mk_traj(action))  # send commands
        else:
            self.get_logger().warn('Safety limit exceeded, switching to baseline.')
            self.cmd_pub.publish(safe_controller(msg))  # fallback

    def infer_action(self, js):
        obs = self.build_obs(js)  # convert to tensor
        with torch.no_grad():
            a = self.policy(obs)  # policy outputs torques or targets
        return a.cpu().numpy()

    def check_safety(self, action):
        return (abs(action) < self.safety_limits['torque_max']).all()

def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()