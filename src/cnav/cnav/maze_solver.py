#!/usr/bin/env python3
"""
Robust Maze Solver using Nav2 Simple Commander
- Loads waypoints (x y z qx qy qz qw) from a file
- Visits all waypoints in sequence
- Retries failed waypoints to ensure all are visited
"""

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math, tf_transformations
from geometry_msgs.msg import PoseStamped


def load_waypoints_from_file(filepath, default_z=0.0, frame="map"):
    """
    Accepts any of the following per-line formats (whitespace-separated):

      1)  x  y                                     (planar goal, yaw = 0)
      2)  x  y  yaw_deg                            (planar goal, explicit yaw)
      3)  x  y  z  qx  qy  qz  qw                  (full 3-D pose)

    • Blank lines and lines starting with ‘#’ are ignored.
    • Malformed lines are skipped with a warning.
    """
    waypoints = []

    with open(filepath, "r") as file:
        for idx, raw in enumerate(file, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            try:
                if len(parts) == 2:                       # x y
                    x, y = map(float, parts)
                    z = default_z
                    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, 0)

                elif len(parts) == 3:                     # x y yaw_deg
                    x, y, yaw_deg = map(float, parts)
                    z = default_z
                    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
                        0, 0, math.radians(yaw_deg)
                    )

                elif len(parts) == 7:                     # x y z qx qy qz qw
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


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Initial pose setup
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.2
    initial_pose.pose.orientation.z = 0.0031688984369330486
    initial_pose.pose.orientation.w = 0.9999949790287431
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Load waypoints
    waypoint_file = "/home/karuppia/autonomous_tb3_ws/src/autonomous_tb3/autonomous_tb3/way.txt"
    goals = load_waypoints_from_file(waypoint_file)

    if not goals:
        print("[ERROR] No valid waypoints loaded.")
        return

    for i, goal in enumerate(goals):
        print(f"[INFO] Navigating to waypoint {i+1}/{len(goals)}...")
        success = False
        attempts = 0
        max_attempts = 3

        while not success and attempts < max_attempts:
            navigator.goToPose(goal)
            while not navigator.isTaskComplete():
                pass

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"[✓] Waypoint {i+1} reached successfully.")
                success = True
            else:
                attempts += 1
                print(f"[!] Waypoint {i+1} attempt {attempts} failed. Retrying...")

        if not success:
            print(f"[X] Failed to reach waypoint {i+1} after {max_attempts} attempts. Exiting.")
            break

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
