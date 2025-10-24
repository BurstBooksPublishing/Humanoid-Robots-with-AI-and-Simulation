import rclpy
from rclpy.node import Node
# import QP solver and message types (placeholders)
from qp_solver import solve_qp  # # custom QP solver wrapper
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
# node publishes safe velocity commands
class ProxemicCBF(Node):
    def __init__(self):
        super().__init__('proxemic_cbf')
        self.sub_pose = self.create_subscription(
            Float32, 'human_distance', self.pose_cb, 10)  # # distance in meters
        self.sub_affect = self.create_subscription(
            Float32, 'human_discomfort', self.affect_cb, 10)  # # [0,1]
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        self.d = 2.0; self.e = 0.0  # # init state
    def pose_cb(self, msg):
        self.d = msg.data
    def affect_cb(self, msg):
        self.e = msg.data
    def control_loop(self):
        # desired forward velocity (task)
        v_des = 0.5
        # comfort cost gradient approximation (minimize C ~ 1/(1+d) + e)
        C = 1.0/(1.0 + self.d) + self.e
        # CBF: h = d - d_safe
        d_safe = 0.6
        h = self.d - d_safe
        # Linearized CBF constraint: a*u <= b
        # here u is scalar forward velocity; dot h = -u (approach reduces distance)
        kappa = 3.0
        A = [[-1.0]]  # -u <= -kappa*h  => -u + kappa*h <= 0
        b = [-kappa*h]
        # QP: minimize (u - v_des)^2 + lambda * C * u^2
        Q = [[1.0 + 0.5*C]]; c = [-v_des]
        u = solve_qp(Q, c, A, b, lb=[0.0], ub=[1.0])  # # respect bounds
        cmd = Twist()
        cmd.linear.x = float(u[0])
        self.pub_cmd.publish(cmd)
def main():
    rclpy.init()
    node = ProxemicCBF()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()