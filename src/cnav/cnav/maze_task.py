#!/usr/bin/env python3
"""
Colour detector for TurtleBot3 + ROS 2.
Prints a line when a red/blue object is both large enough and within 0.85 m.
"""

import math, cv2, numpy as np, rclpy
from rclpy.node         import Node
from sensor_msgs.msg    import Image, LaserScan
from cv_bridge          import CvBridge

# ------------------------------ Tunable Parameters ------------------------------
HSV_BLUE = ((100,130, 50), (140,255,255))
HSV_RED1 = ((  0,100,100), ( 10,255,255))
HSV_RED2 = ((160,100,100), (179,255,255))
MIN_PIX_AREA    = 100000
REPORT_DISTANCE = 1.0
CAMERA_HFOV_DEG = 77     # Horizontal FOV of the RGB camera
# ---------------------------------------------------------------------------------

class ColourDetector(Node):
    def __init__(self):
        super().__init__("colour_detector")
        self.bridge = CvBridge()
        self.scan   = None

        self.create_subscription(Image,     "/camera/image_raw", self.img_cb,   10)
        self.create_subscription(LaserScan, "/scan",              self.scan_cb,  10)

    def scan_cb(self, msg):
        self.scan = msg

    def img_cb(self, msg):
        if self.scan is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        masks = {
            "blue": cv2.inRange(hsv, *HSV_BLUE),
            "red":  cv2.inRange(hsv, *HSV_RED1) | cv2.inRange(hsv, *HSV_RED2)
        }

        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            M = cv2.moments(largest)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Estimate angle of object in image FOV
            img_w = img.shape[1]
            norm_x = (cx - img_w / 2) / (img_w / 2)
            angle = norm_x * math.radians(CAMERA_HFOV_DEG / 2)

            scan = self.scan
            angle_min = scan.angle_min
            angle_inc = scan.angle_increment
            n_beams = len(scan.ranges)

            beam_idx = int(round((angle - angle_min) / angle_inc)) % n_beams
            neighbors = [scan.ranges[(beam_idx + i) % n_beams] for i in (-2, -1, 0, 1, 2)]
            dist = np.nanmedian(np.array(neighbors))

            self.get_logger().info(
                f"[{colour.upper()} — Area: {int(area)} px, "
                f"Distance: {dist:.2f} m (beam {beam_idx})"
            )

            # Only check filtering criteria now
            if area >= MIN_PIX_AREA and 0.1 < dist < REPORT_DISTANCE and dist <= scan.range_max:
                self.get_logger().info(
                    f"Selected - Detected {colour} object at {dist:.2f} m "
                    f"(beam {beam_idx}) — {int(area)} px"
                )

def main(args=None):
    rclpy.init(args=args)
    node = ColourDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
