import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class ColorFollower(Node):
    def __init__(self):
        super().__init__('color_follower')

        # Declare parameters
        self.declare_parameter('color_topic', '/image_raw')
        self.declare_parameter('depth_topic', '/depth/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_mps', 0.2)
        self.declare_parameter('max_angular_rps', 0.8)
        self.declare_parameter('k_turn', 0.003)
        self.declare_parameter('k_distance', 0.5)
        self.declare_parameter('target_distance_cm', 1.0)
        self.declare_parameter('min_bbox_area', 500)

        # Get parameters
        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.max_linear = self.get_parameter('max_linear_mps').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_rps').get_parameter_value().double_value
        self.k_turn = self.get_parameter('k_turn').get_parameter_value().double_value
        self.k_distance = self.get_parameter('k_distance').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance_cm').get_parameter_value().double_value
        self.min_bbox_area = self.get_parameter('min_bbox_area').get_parameter_value().integer_value

        # State variables
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_scan = None
        self.yellow_detected = False

        # Publishers and subscribers
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, 10)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        
        self.get_logger().info(f"[Color Follower] Subscribed to {self.color_topic}, {self.depth_topic} and {self.scan_topic}")
        self.get_logger().info(f"[Color Follower] Publishing to {self.cmd_topic}")
        self.get_logger().info(f"[Color Follower] Target distance: {self.target_distance}m")

    def on_scan(self, msg: LaserScan):
        """Store latest laser scan data"""
        self.latest_scan = msg

    def on_depth(self, msg: Image):
        """Store latest depth image"""
        self.latest_depth = msg

    def on_color(self, msg: Image):
        try:
            color_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")
            return

        # Convert to HSV and detect yellow vest
        hsv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([60, 70, 45])
        upper_yellow = np.array([75, 120, 100])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            bbox = cv2.boundingRect(c)
            x, y, w, h = bbox
            
            if w * h < self.min_bbox_area:
                self.yellow_detected = False
                self.pub_cmd.publish(twist)
                return

            self.yellow_detected = True
            height, width, _ = color_cv.shape
            cx = x + w / 2
            error_x = cx - width / 2

            # Calculate angular velocity to center the target
            twist.angular.z = -self.k_turn * error_x
            twist.angular.z = float(np.clip(twist.angular.z, -self.max_angular, self.max_angular))

            # Calculate linear velocity based on depth to the target
            if self.latest_depth is not None:
                try:
                    depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
                except Exception as e:
                    self.get_logger().error(f"Depth conversion error: {e}")
                    twist.linear.x = 0.0
                else:
                    roi = depth_cv[y:y+h, x:x+w]
                    valid_depths = roi[np.isfinite(roi) & (roi > 0.1)]
                    
                    if len(valid_depths) > 0:
                        dist = np.median(valid_depths)
                        distance_error = dist - self.target_distance
                        
                        twist.linear.x = self.k_distance * distance_error
                        twist.linear.x = float(np.clip(twist.linear.x, 0.0, self.max_linear))  # No backing up, only forward or stop
                        
                        self.get_logger().info(
                            f"Vest detected | Dist: {dist:.2f}m | Err: {distance_error:.2f}m | "
                            f"AngErr: {error_x:.0f}px | v={twist.linear.x:.2f}, w={twist.angular.z:.2f}",
                            throttle_duration_sec=0.5
                        )
                    else:
                        twist.linear.x = 0.0
                        self.get_logger().warn("No valid depth data in ROI", throttle_duration_sec=2.0)
            else:
                twist.linear.x = 0.0
                self.get_logger().warn("No depth image received yet", throttle_duration_sec=2.0)

            # Safety check with lidar: stop forward movement if anything is within target distance in front
            if self.latest_scan is not None:
                ranges = np.array(self.latest_scan.ranges)
                ranges[~np.isfinite(ranges)] = np.inf
                
                angle_min = self.latest_scan.angle_min
                inc = self.latest_scan.angle_increment
                angles = angle_min + np.arange(len(ranges)) * inc
                angles = (angles + np.pi) % (2 * np.pi) - np.pi  # Normalize to -pi to pi
                
                front_angle = np.radians(30)  # 60 degrees total front cone
                front_mask = np.abs(angles) < front_angle
                
                front_ranges = ranges[front_mask]
                valid_front = front_ranges[(front_ranges > 0.1) & (front_ranges < np.inf)]
                
                if len(valid_front) > 0:
                    min_d = np.min(valid_front)
                    if min_d <= self.target_distance:
                        twist.linear.x = 0.0
                        self.get_logger().info(f"Safety stop: Lidar detects obstacle at {min_d:.2f}m", throttle_duration_sec=0.5)
        else:
            self.yellow_detected = False
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