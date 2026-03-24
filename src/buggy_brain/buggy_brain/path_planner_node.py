#!/usr/bin/env python3
"""
path_planner_node.py — Team Bravo v3 + Team Charlie integration
Publishes /planned_path, /navigation_command, /planning_status
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
    EDGES, NODES, DESTINATION_DISPLAY,
    find_shortest_path, VALID_DESTINATIONS, MENU
)

    EDGES, NODES, DESTINATION_DISPLAY, find_shortest_path,
    VALID_DESTINATIONS, MENU
)


START_NODE = 'BUGGY_HUB'


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self._path_pub   = self.create_publisher(Path,   '/planned_path',       10)
        self._cmd_pub    = self.create_publisher(String, '/navigation_command',  10)
        self._status_pub = self.create_publisher(String, '/planning_status',     10)

        self._current_start   = START_NODE
        self._navigating      = False
        self._last_path_nodes = []
        self._target_node     = None
        self._return_timer    = None
        self._lock            = threading.Lock()

        self.create_subscription(
            String, '/navigation_command',  self._nav_cmd_callback,       10)
        self.create_subscription(
            String, '/goal_destination',    self._goal_destination_cb,    10)
        self.create_subscription(
            String, '/destination_request', self._destination_request_cb, 10)

        self.create_timer(2.0, self._republish_path)

        self.get_logger().info('PathPlannerNode started.')
        self.get_logger().info(f'Current position: {self._current_start}')

        self._path_pub = self.create_publisher(Path, '/planned_path', 10)
        self._cmd_pub  = self.create_publisher(String, '/navigation_command', 10)

        self._current_start = START_NODE
        self._navigating    = False
        self._last_path_nodes = []
        self._target_node: str | None = None
        self._return_timer = None
        self._lock = threading.Lock()

        self.create_subscription(
            String, '/navigation_command', self._nav_cmd_callback, 10)

        # Accept destination from external topic (e.g., from set_destination CLI script)
        self.create_subscription(
            String, '/destination_request', self._destination_request_cb, 10)

        # Re-publish path every 2 s so a late-starting waypoint_follower won't miss it
        self.create_timer(2.0, self._republish_path)

        self.get_logger().info('PathPlannerNode started.')

        # CRITICAL: input() runs in daemon thread — never in main ROS spin thread
        self._input_thread = threading.Thread(
            target=self._input_loop, daemon=True, name='destination_input_thread')
        self._input_thread.start()

    def _goal_destination_cb(self, msg: String):
        """Accept goals from /goal_destination topic (Team Charlie interface)."""
        destination = msg.data.strip().upper()
        self.get_logger().info(f'Received goal: {destination}')
        with self._lock:
            if destination not in NODES:
                self.get_logger().warn(f'Unknown destination: {destination}')
                self._publish_status(f'ERROR: Unknown destination {destination}')
                return
            if destination == self._current_start:
                self.get_logger().info('Already at destination.')
                self._publish_status(f'Already at {destination}')
                return
            path_nodes = find_shortest_path(self._current_start, destination)
            if not path_nodes:
                self.get_logger().error(f'No path to {destination}')
                self._publish_status(f'ERROR: No path to {destination}')
                return
            self.get_logger().info(f'Path: {" -> ".join(path_nodes)}')
            waypoints = path_nodes[1:] if len(path_nodes) > 1 else path_nodes
            self._publish_path(waypoints)
            self._last_path_nodes = waypoints
            self._target_node     = destination
            self._navigating      = True
            self._publish_status(f'PLANNING: {" -> ".join(path_nodes)}')

    def _destination_request_cb(self, msg: String):
    def _destination_request_cb(self, msg: String):
        """Handle destination set via /destination_request topic."""
        choice = msg.data.strip().upper()
        self.get_logger().info(f'Received topic request: {choice}')
        self._process_destination(choice)

    def _input_loop(self):
        while rclpy.ok():
            try:
                raw = input(MENU)
            except EOFError:
                break
            choice = raw.strip().upper()
            if choice:
                self._process_destination(choice)

    def _process_destination(self, choice: str):
        with self._lock:
            if choice not in VALID_DESTINATIONS:
                print(f"  [INPUT] Invalid choice '{choice}'. Enter A, B, C, or D.")
                return
            if self._navigating:
                print("  [INPUT] Already navigating. Wait for arrival.")
                return
            target_node = VALID_DESTINATIONS[choice]
            path_nodes  = find_shortest_path(self._current_start, target_node)
            if not path_nodes:
                print(f"  [PLANNER] ERROR: No path from {self._current_start} to {target_node}!")
                return
            route_str = ' -> '.join(path_nodes)
            print(f"\n  [PLANNER] Path: {route_str}. Navigating...\n")
            self.get_logger().info(f'Path: {route_str}')
            waypoints = path_nodes[1:] if len(path_nodes) > 1 else path_nodes
            self._publish_path(waypoints)
            self._last_path_nodes = waypoints
            self._target_node     = target_node
            self._navigating      = True
            self._publish_status(f'PLANNING: {route_str}')

    def _nav_cmd_callback(self, msg: String):
        with self._lock:
            if msg.data == 'DESTINATION_REACHED' and self._navigating:
                if self._target_node is not None:
                    self._current_start = self._target_node
                old_target       = self._target_node
                self._navigating = False
                self._last_path_nodes = []
                self._target_node     = None
                print(f"\n  [PLANNER] Reached {DESTINATION_DISPLAY.get(self._current_start, self._current_start)}.")
                self._publish_status(f'ARRIVED: {self._current_start}')
                if old_target is not None and old_target != 'BUGGY_HUB':
                    print("  [PLANNER] Returning to Buggy Hub automatically...")
                    self.get_logger().info('Returning to Hub.')
                    def safe_return():
                        if rclpy.ok():
                            self._process_destination('D')

            if self._navigating:
                print("  [INPUT] Already navigating. Wait for arrival.")
                return

            target_node = VALID_DESTINATIONS[choice]
            path_nodes = find_shortest_path(EDGES, self._current_start, target_node)

            if path_nodes is None or len(path_nodes) == 0:
                print(f"  [PLANNER] ERROR: No path from {self._current_start} to {target_node}!")
                return

            total_dist = sum(
                next((w for neighbor, w in EDGES.get(path_nodes[i], []) if neighbor == path_nodes[i + 1]), 0.0)
                for i in range(len(path_nodes) - 1)
            ) if len(path_nodes) > 1 else 0.0
            
            route_str = ' -> '.join(path_nodes)
            print(f"\n  [PLANNER] Path: {route_str} ({total_dist:.0f} m). Navigating...\n")
            self.get_logger().info(f'Path: {route_str}')

            # Publish path (excluding current start position if possible)
            waypoints = path_nodes[1:] if len(path_nodes) > 1 else path_nodes
            self._publish_path(waypoints)
            self._last_path_nodes = waypoints
            self._target_node = target_node

            nav_cmd = String()
            nav_cmd.data = f'START:{target_node}'
            self._cmd_pub.publish(nav_cmd)
            self._navigating = True

    def _nav_cmd_callback(self, msg: String):
        # Match EXACT string to prevent accidental triggers
        with self._lock:
            if msg.data == 'DESTINATION_REACHED' and self._navigating:
                if self._target_node is not None:
                    target: str = self._target_node
                    self._current_start = target
                
                old_target = self._target_node
                self._navigating = False
                self._last_path_nodes = []
                self._target_node = None # reset
                
                print(f"\n  [PLANNER] Reached {DESTINATION_DISPLAY.get(self._current_start, self._current_start)}.")
                
                # AUTOMATIC RETURN TO HUB
                if old_target is not None and old_target != 'BUGGY_HUB':
                    print("  [PLANNER] MISSION COMPLETE. Returning to Buggy Hub automatically...")
                    self.get_logger().info('Destination reached. Returning to Hub.')
                    
                    def safe_return():
                        if rclpy.ok():
                            self._process_destination('D')
                    
                    if self._return_timer:
                        self._return_timer.cancel()
                    self._return_timer = threading.Timer(1.0, safe_return)
                    self._return_timer.start()
                else:
                    print("  [PLANNER] PARKED AT HUB.")

    def _republish_path(self):
                    if old_target == 'BUGGY_HUB':
                        print("  [PLANNER] PARKED AT HUB. Select next destination:")
                    print(MENU, end='', flush=True)

    def _republish_path(self):
        """Re-send the current path every 2 s so late-starting nodes don't miss it."""
        with self._lock:
            if self._navigating and self._last_path_nodes:
                self._publish_path(self._last_path_nodes)

    def _publish_path(self, node_names: list):
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for name in node_names:
            coords = NODES.get(name)
            if coords is None:
                self.get_logger().warn(f'Unknown node "{name}" — skipping')
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for name in node_names:
            coords = NODES.get(name)
            if coords is None:
                self.get_logger().warn(f'Unknown node "{name}" in path - skipping')
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
        self.get_logger().info(f'Published path: {" -> ".join(node_names)}')

    def _publish_status(self, text: str):
        msg      = String()
        msg.data = text
        self._status_pub.publish(msg)

        self._path_pub.publish(path_msg)
        self.get_logger().info(f'Published path: {" → ".join(node_names)}')

    def destroy_node(self):
        if self._return_timer:
            self._return_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
if __name__ == '__main__':
    main()
