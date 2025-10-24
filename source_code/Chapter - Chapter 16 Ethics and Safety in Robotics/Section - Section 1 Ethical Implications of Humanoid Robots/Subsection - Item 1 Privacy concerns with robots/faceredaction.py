import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FaceRedactor(Node):
    def __init__(self):
        super().__init__('face_redactor')      # node name
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Image, '/camera/image_redacted', 10)
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # convert ROS image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x,y,w,h) in faces:
            roi = img[y:y+h, x:x+w]
            roi = cv2.GaussianBlur(roi, (99,99), 30)  # blur face region
            img[y:y+h, x:x+w] = roi
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(out_msg)  # publish redacted stream

def main(args=None):
    rclpy.init(args=args)
    n = FaceRedactor()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()