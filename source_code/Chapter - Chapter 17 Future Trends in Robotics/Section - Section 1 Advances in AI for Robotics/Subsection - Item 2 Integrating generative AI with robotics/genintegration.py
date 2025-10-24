import rclpy
from rclpy.node import Node
# message types are placeholders for brevity
from sensor_msgs.msg import JointState  # joint commands
from std_msgs.msg import Float32        # confidence

class GenIntegrator(Node):
    def __init__(self):
        super().__init__('gen_integrator')
        self.sub_proposal = self.create_subscription(JointState,'/gen/proposal',self.cb_prop,10)
        self.sub_conf = self.create_subscription(Float32,'/gen/conf',self.cb_conf,10)
        self.pub_safe = self.create_publisher(JointState,'/robot/joint_cmd',10)
        self.nominal = None  # stored nominal controller output
        self.last_conf = 0.0

    def cb_prop(self, msg):
        u_gen = msg.position  # gen proposal
        alpha = self._map_conf_to_alpha(self.last_conf)  # trust scaling
        u_nom = self._get_nominal()  # synchronous request to low-level controller
        u_safe = self._blend_and_check(u_nom, u_gen, alpha)
        if u_safe is not None:
            out = JointState()
            out.position = u_safe
            self.pub_safe.publish(out)
        else:
            # publish fallback safe posture
            fallback = self._fallback_posture()
            out = JointState()
            out.position = fallback
            self.pub_safe.publish(out)

    def cb_conf(self, msg):
        self.last_conf = msg.data

    def _map_conf_to_alpha(self, conf):
        # simple mapping: low confidence -> small alpha
        return max(0.0, min(1.0, (conf - 0.2) / 0.8))

    def _blend_and_check(self, u_nom, u_gen, alpha):
        # blend then validate kinematics, torques, collisions
        u = [(1-alpha)*n + alpha*g for n,g in zip(u_nom,u_gen)]
        if self._collision_check(u): return None
        if not self._joint_limits_ok(u): return None
        return u

    # ... implementations of _get_nominal, _collision_check, _joint_limits_ok, _fallback_posture omitted