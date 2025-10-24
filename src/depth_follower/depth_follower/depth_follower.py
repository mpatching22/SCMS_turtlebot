import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan  # Sensor messages
from geometry_msgs.msg import Twist           # Robot movement command
import numpy as np                            # For numerical operations
import cv2                                    # For computer vision
from cv_bridge import CvBridge                # To convert between ROS and OpenCV images

class ColorFollower(Node):
    def __init__(self):
        super().__init__('color_follower')

        # --- Parameter Declaration ---
        # Declare all configurable parameters with their default values.
        # This allows them to be changed from a launch file or command line.
        self.declare_parameter('color_topic', '/image_raw')         # Topic for the main color camera
        self.declare_parameter('depth_topic', '/depth/image_raw')   # Topic for the depth camera
        self.declare_parameter('scan_topic', '/scan')               # Topic for the 2D LiDAR scanner
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')         # Topic to publish movement commands
        self.declare_parameter('max_linear_mps', 0.2)           # Max forward speed in meters/sec
        self.declare_parameter('max_angular_rps', 0.8)          # Max turning speed in radians/sec
        self.declare_parameter('k_turn', 0.003)                 # Proportional gain for turning (pixels to rad/s)
        self.declare_parameter('k_distance', 0.5)               # Proportional gain for distance (meters to m/s)
        self.declare_parameter('target_distance_cm', 1.0)       # Target stopping distance in meters
        self.declare_parameter('min_bbox_area', 500)            # Smallest pixel area to be considered a valid target

        # --- Parameter Retrieval ---
        # Get the actual values of the parameters (from defaults or user overrides)
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

        # --- State Variables ---
        # Initialize variables to store data from callbacks
        self.bridge = CvBridge()             # Tool to convert ROS Image messages to OpenCV format
        self.latest_depth = None             # Stores the most recent depth image message
        self.latest_scan = None              # Stores the most recent laser scan message
        self.yellow_detected = False         # Flag to track if the target is currently visible

        # --- Publishers and Subscribers ---
        # Create a publisher to send Twist (velocity) commands to the robot
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        
        # Create subscribers to receive data from sensors.
        # The 'on_color' callback is the main control loop, triggered by new camera frames.
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, 10)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        
        # Log to the console that the node has started and its configuration
        self.get_logger().info(f"[Color Follower] Subscribed to {self.color_topic}, {self.depth_topic} and {self.scan_topic}")
        self.get_logger().info(f"[Color Follower] Publishing to {self.cmd_topic}")
        self.get_logger().info(f"[Color Follower] Target distance: {self.target_distance}m")

    # --- Callback Functions ---

    def on_scan(self, msg: LaserScan):
        """Callback to store the latest laser scan data."""
        self.latest_scan = msg

    def on_depth(self, msg: Image):
        """Callback to store the latest depth image data."""
        self.latest_depth = msg

    # This is the main control loop, triggered by every new color camera frame
    def on_color(self, msg: Image):
        # --- 1. Image Processing ---
        try:
            # Convert the ROS Image message to an OpenCV image (BGR format)
            color_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")
            return  # Skip this frame if conversion fails

        # --- 2. Color Detection (Yellow) ---
        # Convert the BGR image to HSV (Hue, Saturation, Value) color space
        hsv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2HSV)
        # Define the HSV range for the yellow vest (These values are for a greenish-yellow)
        lower_yellow = np.array([60, 70, 45])
        upper_yellow = np.array([75, 120, 100])
        # Create a binary mask where yellow pixels are white and all others are black
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # Find all distinct shapes (contours) in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize a new, empty Twist command (all velocities are 0.0)
        twist = Twist()

        # --- 3. Target Found Logic ---
        # Check if any contours (yellow shapes) were found
        if len(contours) > 0:
            # Find the largest contour by area (assuming this is our target)
            c = max(contours, key=cv2.contourArea)
            # Get the bounding box (x, y, width, height) of this largest contour
            bbox = cv2.boundingRect(c)
            x, y, w, h = bbox
            
            # --- 3a. Filter by Size ---
            # If the detected object is smaller than the minimum area, ignore it (it's likely noise)
            if w * h < self.min_bbox_area:
                self.yellow_detected = False
                self.pub_cmd.publish(twist)  # Publish the empty (stop) command
                return  # Stop processing this frame

            # --- 3b. Target Locked ---
            self.yellow_detected = True
            # Get the dimensions of the camera image
            height, width, _ = color_cv.shape
            # Find the horizontal center of the bounding box
            cx = x + w / 2
            # Calculate the "error" - how far the target is from the center of the screen (in pixels)
            error_x = cx - width / 2

            # --- 3c. Calculate Angular (Turning) Velocity ---
            # Use a proportional controller (P-controller) to turn
            # The further the target is from the center, the faster the robot turns
            twist.angular.z = -self.k_turn * error_x
            # Clamp the turning speed to the maximum allowed value
            twist.angular.z = float(np.clip(twist.angular.z, -self.max_angular, self.max_angular))

            # --- 3d. Calculate Linear (Forward) Velocity using Depth Camera ---
            # Only calculate distance if we have a valid depth image from the on_depth callback
            if self.latest_depth is not None:
                try:
                    # Convert the ROS depth image to an OpenCV array
                    # 'passthrough' keeps the native encoding (e.g., 32-bit float in meters)
                    depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
                except Exception as e:
                    self.get_logger().error(f"Depth conversion error: {e}")
                    twist.linear.x = 0.0  # Stop if conversion fails
                else:
                    # --- 3e. Get Distance from Depth Image ---
                    # Get the rectangular "Region of Interest" (ROI) from the depth image
                    # This ROI corresponds exactly to the yellow bounding box
                    roi = depth_cv[y:y+h, x:x+w]
                    # Filter out invalid depth values (like 'inf', 'NaN', or 0)
                    valid_depths = roi[np.isfinite(roi) & (roi > 0.1)]
                    
                    if len(valid_depths) > 0:
                        # Find the median depth value within the box (more robust to outliers than mean)
                        dist = np.median(valid_depths)
                        # Calculate the distance error: how far we are from our target distance
                        distance_error = dist - self.target_distance
                        
                        # Use a P-controller for linear speed
                        # The further we are, the faster we move
                        twist.linear.x = self.k_distance * distance_error
                        # Clamp the speed between 0.0 (stop) and max_linear.
                        # This prevents the robot from backing up if it overshoots.
                        twist.linear.x = float(np.clip(twist.linear.x, 0.0, self.max_linear))
                        
                        # Log all debug info (throttled to 0.5s to avoid spam)
                        self.get_logger().info(
                            f"Vest detected | Dist: {dist:.2f}m | Err: {distance_error:.2f}m | "
                            f"AngErr: {error_x:.0f}px | v={twist.linear.x:.2f}, w={twist.angular.z:.2f}",
                            throttle_duration_sec=0.5
                        )
                    else:
                        # If the bounding box has no valid depth data (e.g., target is too far/close)
                        twist.linear.x = 0.0 # Stop
                        self.get_logger().warn("No valid depth data in ROI", throttle_duration_sec=2.0)
            else:
                # If we haven't received any depth images yet
                twist.linear.x = 0.0 # Stop
                self.get_logger().warn("No depth image received yet", throttle_duration_sec=2.0)

            # --- 4. LiDAR Safety Override ---
            # This check overrides all other logic for a hard safety stop.
            # It uses the 360-degree LiDAR as a failsafe.
            if self.latest_scan is not None:
                # Get the raw ranges from the LiDAR
                ranges = np.array(self.latest_scan.ranges)
                # Replace invalid 'NaN' or 'inf' with a large number (infinity)
                ranges[~np.isfinite(ranges)] = np.inf
                
                # --- 4a. Get Front-Facing Arc ---
                # Get the angle for each ray in the scan
                angle_min = self.latest_scan.angle_min
                inc = self.latest_scan.angle_increment
                angles = angle_min + np.arange(len(ranges)) * inc
                # Normalize angles to be from -pi to +pi (where 0 is front)
                angles = (angles + np.pi) % (2 * np.pi) - np.pi
                
                # Define our "front" cone (30 degrees to the left and 30 to the right)
                front_angle = np.radians(30)
                # Create a boolean mask for all rays within this front cone
                front_mask = np.abs(angles) < front_angle
                
                # Get only the ranges from the front-facing rays
                front_ranges = ranges[front_mask]
                # Filter out any remaining invalid (e.g., 0) values
                valid_front = front_ranges[(front_ranges > 0.1) & (front_ranges < np.inf)]
                
                # --- 4b. Perform Safety Stop ---
                if len(valid_front) > 0:
                    # Find the closest object in our path
                    min_d = np.min(valid_front)
                    # If that object is closer than our target stopping distance...
                    if min_d <= self.target_distance:
                        # ...force the robot to stop moving forward, regardless of what the
                        # depth camera says. (Turning is still allowed).
                        twist.linear.x = 0.0 
                        self.get_logger().info(f"Safety stop: Lidar detects obstacle at {min_d:.2f}m", throttle_duration_sec=0.5)
        
        # --- 5. No Target Found Logic ---
        else:
            # If no yellow contours were found (or they were too small)
            self.yellow_detected = False
            # Log this (throttled to 2s to avoid spam)
            self.get_logger().info("No yellow target detected", throttle_duration_sec=2.0)
            # The twist message is still all zeros, so the robot will stop.

        # --- 6. Publish Command ---
        # Send the final Twist message (either with velocity or all zeros) to the robot
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
