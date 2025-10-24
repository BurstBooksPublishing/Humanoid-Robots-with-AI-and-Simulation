#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# simple parameters
RESID_THRESH = 0.05    # rad, residual threshold
TEMP_THRESH = 65.0     # degC, temperature alert
ALPHA = 0.02           # EWMA factor

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diag_node')
        self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        self.ema_temps = {}   # persistent EWMA per joint
    def cb_joint(self, msg):
        # msg.position, msg.velocity, msg.effort correspond to joints
        for i,name in enumerate(msg.name):
            pos = msg.position[i]
            effort = msg.effort[i] if i < len(msg.effort) else 0.0
            temp = self.estimate_temp_from_effort(name, effort)  # model-based proxy
            # update EWMA
            prev = self.ema_temps.get(name, temp)
            ema = ALPHA*temp + (1-ALPHA)*prev
            self.ema_temps[name] = ema
            # simple residual check against commanded position if available
            # assume commanded position in header frame or separate topic in practice
            # here we use a placeholder commanded pos = 0 for demonstration
            cmd = 0.0
            residual = abs(pos - cmd)
            if residual > RESID_THRESH:
                self.get_logger().warn(f'Large residual {residual:.3f} rad on {name}')
            if ema > TEMP_THRESH:
                self.get_logger().error(f'High motor temp {ema:.1f}C on {name}')
    def estimate_temp_from_effort(self, name, effort):
        # simple linear proxy: temp increases with RMS effort (placeholder)
        return 30.0 + 5.0*abs(effort)

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()