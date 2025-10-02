import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np

class DepthFollower(Node):
    def __init__(self):
        super().__init__('depth_follower')

        # Params (match your YAML if present)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_distance_m', 1.0)
        self.declare_parameter('max_linear_mps', 0.5)
        self.declare_parameter('max_angular_rps', 1.5)
        self.declare_parameter('k_v', 0.8)
        self.declare_parameter('k_w', 0.003)
        self.declare_parameter('max_range_m', 4.0)

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Publisher
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        # Subscriber (raw Image; we parse as 32FC1 or 16UC1 later if needed)
        self.sub_depth = self.create_subscription(Image, depth_topic, self.on_depth, 10)

        self.get_logger().info(f"DepthFollower up. Subscribing: {depth_topic} → Publishing: {cmd_topic}")

    def on_depth(self, msg: Image):
        # Very safe MVP: just publish zero until we wire cv_bridge later
        # This proves the node runs and parameters load.
        self.pub_cmd.publish(Twist())

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
