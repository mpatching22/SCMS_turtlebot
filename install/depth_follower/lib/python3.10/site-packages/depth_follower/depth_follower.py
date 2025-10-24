import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class ColorFollower(Node):
    def __init__(self):
        super().__init__('color_follower')

        self.declare_parameter('color_topic', '/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_mps', 0.2)
        self.declare_parameter('max_angular_rps', 0.8)
        self.declare_parameter('k_turn', 0.003)
        self.declare_parameter('min_bbox_area', 500)

        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.max_linear = self.get_parameter('max_linear_mps').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_rps').get_parameter_value().double_value
        self.k_turn = self.get_parameter('k_turn').get_parameter_value().double_value
        self.min_bbox_area = self.get_parameter('min_bbox_area').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, 10)
        self.get_logger().info(f"[Color Follower] Subscribed to {self.color_topic} publishing to {self.cmd_topic}")

    def on_color(self, msg: Image):
        try:
            color_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")
            return

        hsv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            bbox = cv2.boundingRect(c)
            x, y, w, h = bbox
            if w * h < self.min_bbox_area:
                self.pub_cmd.publish(twist)
                return

            height, width, _ = color_cv.shape
            cx = x + w / 2
            error_x = cx - width / 2

            twist.linear.x = self.max_linear
            twist.angular.z = -self.k_turn * error_x
            twist.angular.z = float(np.clip(twist.angular.z, -self.max_angular, self.max_angular))
            self.get_logger().info(
                f"Yellow detected | Angular err: {error_x:.0f}px | v={twist.linear.x:.2f}, w={twist.angular.z:.2f}",
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info("No yellow target detected", throttle_duration_sec=2.0)
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
