import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import spidev, time, struct

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_pub')
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        # SPI init for onboard IMU (example) -- adjust per hardware.
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)               # bus, device
        self.spi.max_speed_hz = 1000000
        self.bias_gyro = [0.0, 0.0, 0.0]  # simple bias estimate
        self.timer = self.create_timer(0.005, self.timer_cb) # 200 Hz

    def read_imu(self):
        # raw read; replace with sensor-specific protocol
        raw = self.spi.xfer2([0x00]*12)
        ax, ay, az, gx, gy, gz = struct.unpack('>6h', bytes(raw))
        return [a*1e-3 for a in (ax, ay, az)], [g*1e-3 for g in (gx, gy, gz)]

    def timer_cb(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()  # hardware timestamp preferable
        acc, gyro = self.read_imu()
        # subtract precomputed bias -- simple compensation
        gyro = [g - b for g, b in zip(gyro, self.bias_gyro)]
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro
        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()