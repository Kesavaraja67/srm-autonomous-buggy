#!/usr/bin/env python3
"""
path_planner_node.py  —  Team Bravo
Master Plan §6.2
Publishes: /planned_path (nav_msgs/Path), /navigation_command (std_msgs/String)
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from buggy_brain.map_graph import (
    EDGES, NODES, DESTINATION_NAMES, find_shortest_path, path_distance
)

VALID_DESTINATIONS = {
    'A': 'Main Gate',
    'B': 'Library Block',
    'C': 'Admin Block',
}

MENU = (
    "\n"
    "========================================\n"
    "  SRM Autonomous Campus Buggy\n"
    "  Select Destination:\n"
    "    (A) Main Gate\n"
    "    (B) Library Block\n"
    "    (C) Admin Block\n"
    "  Type a letter and press Enter: "
)

START_NODE = 'START'


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self._path_pub = self.create_publisher(Path, '/planned_path', 10)
        self._cmd_pub  = self.create_publisher(String, '/navigation_command', 10)

        self._current_start = START_NODE
        self._navigating    = False

        self.create_subscription(
            String, '/navigation_command', self._nav_cmd_callback, 10)

        self.get_logger().info('PathPlannerNode started.')

        # CRITICAL: input() runs in daemon thread — never in main ROS spin thread
        self._input_thread = threading.Thread(
            target=self._input_loop, daemon=True, name='destination_input_thread')
        self._input_thread.start()

    def _input_loop(self):
        while rclpy.ok():
            try:
                raw = input(MENU)
            except EOFError:
                break

            choice = raw.strip().upper()

            if choice not in VALID_DESTINATIONS:
                print(f"  [INPUT] Invalid choice '{raw}'. Enter A, B, or C.")
                continue

            if self._navigating:
                print("  [INPUT] Already navigating. Wait for arrival.")
                continue

            path_nodes = find_shortest_path(EDGES, self._current_start, choice)

            if path_nodes is None:
                print(f"  [PLANNER] ERROR: No path from {self._current_start} to {choice}!")
                continue

            dist = path_distance(path_nodes)
            route_str = ' → '.join(path_nodes)
            print(f"\n  [PLANNER] Shortest Path: {route_str}  ({dist:.0f} m). Navigating…\n")

            self._publish_path(path_nodes)

            nav_cmd = String()
            nav_cmd.data = f'START:{choice}'
            self._cmd_pub.publish(nav_cmd)
            self._navigating = True

    def _nav_cmd_callback(self, msg: String):
        if msg.data.startswith('DESTINATION_REACHED'):
            parts = msg.data.split(':')
            reached = parts[1] if len(parts) > 1 else '?'
            self._current_start = reached
            self._navigating = False
            print(f"\n  [PLANNER] Reached {DESTINATION_NAMES.get(reached, reached)}. Select next:")

    def _publish_path(self, node_names: list):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for name in node_names:
            coords = NODES.get(name)
            if coords is None:
                continue
            pose = PoseStamped()
            pose.header.stamp    = path_msg.header.stamp
            pose.header.frame_id = 'odom'
            pose.pose.position.x = coords[0]
            pose.pose.position.y = coords[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self._path_pub.publish(path_msg)
        self.get_logger().info(f'Published path: {" → ".join(node_names)}')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
