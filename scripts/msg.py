#!/usr/bin/env python3
"""
TurtleBot3 Maze Navigator + Real-Time Colour Detector (ROS 2, Correctly Integrated)

* Non-blocking waypoint navigation using Timer
* Real-time red/blue detection using RGB + LiDAR
* Logs last detection near each waypoint
* Added checker for side shift completion
"""

import math, cv2, numpy as np, rclpy, time
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped

# ------------------------------ Tunable Parameters ------------------------------

HSV_BLUE = ((100,130, 50), (140,255,255))
HSV_RED1 = ((  0,100,100), ( 10,255,255))
HSV_RED2 = ((160,100,100), (179,255,255))
MIN_PIX_AREA    = 100000
REPORT_DISTANCE = 1.2
CAMERA_HFOV_DEG = 77
WAYPOINT_FILE   = "/home/karuppia/autonomous_tb3_ws/src/autonomous_tb3/autonomous_tb3/waypoints.txt"

# ---------------------------------------------------------------------------------

def lateral_shift(D, A, W=640, FOV=77):
    """
    Calculate the lateral shift required to avoid an object based on its area
    in the camera image and the distance to the object.
    """
    FOV_rad = math.radians(FOV)
    PPM = W / (2 * math.tan(FOV_rad / 2) * D)
    object_width_pixels = math.sqrt(A)
    offset_pixels = abs(object_width_pixels / 2 - W / 2)
    lateral_shift_meters = offset_pixels / PPM
    return lateral_shift_meters

def load_waypoints_from_file(filepath, default_z=0.0, frame="map"):
    """
    Loads waypoints from a file.
    """
    waypoints = []
    with open(filepath, "r") as file:
        for idx, raw in enumerate(file, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            try:
                if len(parts) == 2:  # x y
                    x, y = map(float, parts)
                    z = default_z
                    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, 0)
                elif len(parts) == 3:  # x y yaw_deg
                    x, y, yaw_deg = map(float, parts)
                    z = default_z
                    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))
                elif len(parts) == 7:  # x y z qx qy qz qw
                    x, y, z, qx, qy, qz, qw = map(float, parts)
                else:
                    raise ValueError("unsupported field count")

                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                waypoints.append(pose)

            except Exception as e:
                print(f"[WARN]  line {idx}: '{line}'  → skipped ({e})")

    print(f"[INFO] Loaded {len(waypoints)} waypoint(s) from {filepath}")
    return waypoints

class MazeNavigatorWithColourDetector(Node):
    def __init__(self):
        super().__init__("maze_colour_node")
        self.bridge = CvBridge()
        self.scan = None
        self.latest_image = None
        self.navigator = BasicNavigator()
        self.goals = load_waypoints_from_file(WAYPOINT_FILE)
        self.i = 0
        self.goal_active = False
        self.last_detection = None
        self.detection_counter = 0
        self.detection_color = None
        self.detection_time = None
        self.detection_confirmed = False
        self.goal_complete_time = None
        self.waiting_for_colour_check = False
        self.performing_side_step = False
        self.current_pose = None
        self.side_step_target = None  # Added to store side step target pose

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.timer = self.create_timer(0.2, self.tick)

    def pose_callback(self, msg):
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan = msg
        self._try_process_detection()

    def image_callback(self, msg):
        self.latest_image = msg
        self._try_process_detection()

    def _try_process_detection(self):
        if self.latest_image is None or self.scan is None:
            return
        self._process_colour_detection()

    def _process_colour_detection(self):
        if self.latest_image is None or self.scan is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        masks = {
            "blue": cv2.inRange(hsv, *HSV_BLUE),
            "red":  cv2.inRange(hsv, *HSV_RED1) | cv2.inRange(hsv, *HSV_RED2)
        }

        current_time = self.get_clock().now().nanoseconds / 1e9

        detected = False

        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < MIN_PIX_AREA:
                continue

            M = cv2.moments(largest)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            img_w = img.shape[1]
            norm_x = (cx - img_w / 2) / (img_w / 2)
            angle = norm_x * math.radians(CAMERA_HFOV_DEG / 2)

            angle_min = self.scan.angle_min
            angle_inc = self.scan.angle_increment
            n_beams = len(self.scan.ranges)

            beam_idx = int(round((angle - angle_min) / angle_inc)) % n_beams
            neighbors = [self.scan.ranges[(beam_idx + i) % n_beams] for i in (-2, -1, 0, 1, 2)]
            dist = np.nanmedian(np.array(neighbors))

            if not (0.1 < dist < REPORT_DISTANCE and dist <= self.scan.range_max):
                continue

            detected = True
            if getattr(self, "staging_colour", None) == colour:
                self.staging_count += 1
                duration = current_time - self.staging_start
            else:
                self.staging_colour = colour
                self.staging_count = 1
                self.staging_start = current_time
                duration = 0.0

            self.get_logger().info(f"[STAGING] {colour.upper()} seen {self.staging_count}x, {duration:.1f}s...")

            if self.staging_count >= 5 and duration >= 0.2:
                self.get_logger().info(
                    f"[TRIGGER] {colour.upper()} detected {self.staging_count}x for {duration:.2f}s → CONFIRMED"
                )
                self.last_detection = (colour.upper(), int(area), round(dist, 2), beam_idx)
                self.staging_colour = None
                self.staging_count = 0
                self.staging_start = None
            return

        if not detected:
            self.staging_colour = None
            self.staging_count = 0
            self.staging_start = None

    def _initiate_side_step(self, go_left=True, step_distance=0.4):
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available. Skipping side-step.")
            return

        pose = self.current_pose.pose
        q = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        side = left if go_left else -left

        start_xy = np.array([pose.position.x, pose.position.y])
        target_xy = start_xy + side * step_distance

        step_pose = PoseStamped()
        step_pose.header.frame_id = "map"
        step_pose.pose.position.x = target_xy[0]
        step_pose.pose.position.y = target_xy[1]
        step_pose.pose.orientation = pose.orientation

        self.side_step_target = step_pose  # Store the target pose

        print(f"[ACTION] Performing side-step {'left' if go_left else 'right'} by {step_distance:.2f} m...")
        self.navigator.goToPose(step_pose)
        self.goal_active = True
        self.performing_side_step = True

    def _make_initial_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.2
        pose.pose.orientation.z = 0.0031688984369330486
        pose.pose.orientation.w = 0.9999949790287431
        return pose

    def tick(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Finish side-step with position check
        if getattr(self, "performing_side_step", False):
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    if self.current_pose and self.side_step_target:
                        current_pos = self.current_pose.pose.position
                        target_pos = self.side_step_target.pose.position
                        distance = math.sqrt(
                            (current_pos.x - target_pos.x)**2 +
                            (current_pos.y - target_pos.y)**2
                        )
                        if distance < 0.1:  # Tolerance of 0.1 meters
                            print("[✓] Side-step completed successfully and position is accurate.")
                        else:
                            print(f"[!] Side-step completed but position is not accurate. Distance: {distance:.2f} m")
                    else:
                        print("[!] Cannot check position accuracy: pose information missing.")
                    self.performing_side_step = False
                    self.i += 1
                    self.goal_active = False
                    self.side_step_target = None  # Reset for next side step
                else:
                    print("[!] Side-step failed.")
                    self.performing_side_step = False
                    self.i += 1  # Proceed anyway as per original behavior
                    self.goal_active = False
            return

        # After reaching a goal, wait to check for colour
        if self.waiting_for_colour_check:
            if self.last_detection:
                colour, area, dist, beam = self.last_detection
                offset = lateral_shift(D=dist, A=area)
                print(f"    └─ Last detection → [{colour}] Area: {area} px, Distance: {dist} m (beam {beam})")
                if colour in ["RED", "BLUE"]:
                    self._initiate_side_step(go_left=(colour == "BLUE"), step_distance=offset)
                    self.last_detection = None
                    self.waiting_for_colour_check = False
                else:
                    self.i += 1
                    self.waiting_for_colour_check = False
            elif self.goal_complete_time and now_sec - self.goal_complete_time > 2.5:
                print("    └─ No color detection near this waypoint.")
                self.i += 1
                self.waiting_for_colour_check = False
            return

        if not self.goal_active:
            if self.i >= len(self.goals):
                print("[✓] All waypoints completed.")
                return

            if self.i == 0:
                self.navigator.setInitialPose(self._make_initial_pose())
                self.navigator.waitUntilNav2Active()

            goal = self.goals[self.i]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position = goal.pose.position
            goal_pose.pose.orientation = goal.pose.orientation
            
            self.navigator.goToPose(goal_pose)
            self.goal_active = True
            print(f"[INFO] Navigating to waypoint {self.i + 1}/{len(self.goals)}...")

        else:
            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"[✓] Waypoint {self.i + 1} reached successfully.")
                self.goal_complete_time = now_sec
                self.waiting_for_colour_check = True
            else:
                print(f"[!] Waypoint {self.i + 1} failed. Retrying...")

            self.goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigatorWithColourDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()