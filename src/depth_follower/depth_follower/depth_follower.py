import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class DepthFollower(Node):
    def __init__(self):
        super().__init__('depth_follower')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_distance_m', 1.0)
        self.declare_parameter('max_linear_mps', 0.5)
        self.declare_parameter('max_angular_rps', 1.5)
        self.declare_parameter('k_v', 0.8)
        self.declare_parameter('k_w', 0.003)
        self.declare_parameter('max_range_m', 4.0)

        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.target_distance = self.get_parameter('target_distance_m').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_mps').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_rps').get_parameter_value().double_value
        self.k_v = self.get_parameter('k_v').get_parameter_value().double_value
        self.k_w = self.get_parameter('k_w').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range_m').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.latest_depth = None

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, 10)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)

        self.get_logger().info(f"RGBD Follower up. Subs: {self.color_topic}, {self.depth_topic} → Pub: {self.cmd_topic}")

    def on_depth(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def on_color(self, msg: Image):
        if self.latest_depth is None:
            return

        color_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()  # Default stop

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            bbox = cv2.boundingRect(c)
            if bbox[2] * bbox[3] < 100:  # Too small
                self.pub_cmd.publish(twist)
                return

            depth_roi = self.latest_depth[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
            valid_depths = depth_roi[np.isfinite(depth_roi)]
            if len(valid_depths) == 0:
                self.pub_cmd.publish(twist)
                return

            d = np.median(valid_depths)
            if d > self.max_range or d < 0.1:
                self.pub_cmd.publish(twist)
                return

            height, width = self.latest_depth.shape
            cx = bbox[0] + bbox[2] / 2
            error_ang = cx - width / 2
            error_dist = d - self.target_distance

            v = self.k_v * error_dist
            w = -self.k_w * error_ang

            twist.linear.x = np.clip(v, -self.max_linear, self.max_linear)
            twist.angular.z = np.clip(w, -self.max_angular, self.max_angular)

        self.pub_cmd.publish(twist)

def main():
    rclpy.init()
    node = DepthFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()