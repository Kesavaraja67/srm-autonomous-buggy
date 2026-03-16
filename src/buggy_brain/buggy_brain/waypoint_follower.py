#!/usr/bin/env python3
"""
waypoint_follower.py  —  Team Bravo
Master Plan §6.3
Subscribes: /planned_path, /odom, /buggy_state
Publishes:  /cmd_vel, /navigation_command
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

# Tuning constants — §6.3
MAX_LINEAR_SPEED  = 4.17   # m/s = 15 km/h
MIN_LINEAR_SPEED  = 0.30   # m/s — always make forward progress
ANGULAR_P_GAIN    = 0.70   # proportional gain on heading error
MAX_ANGULAR_SPEED = 1.00   # rad/s cap
ARRIVAL_RADIUS    = 1.20   # metres — when to advance to next waypoint
SLOW_TURN_SPEED   = 1.00   # m/s linear speed during sharp turn
HEADING_THRESHOLD = 0.30   # rad — sharp turn threshold
HEADING_DEADBAND  = 0.05   # rad — ignore tiny errors, prevents jitter
CONTROL_RATE_HZ   = 10


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollowerNode(Node):

    def __init__(self):
        super().__init__('waypoint_follower_node')

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_pub = self.create_publisher(String, '/navigation_command', 10)

        self.create_subscription(Path,     '/planned_path', self._path_callback,  10)
        self.create_subscription(Odometry, '/odom',         self._odom_callback,  10)
        self.create_subscription(String,   '/buggy_state',  self._state_callback, 10)

        self._waypoints    = []
        self._wp_index     = 0
        self._pos_x        = 0.0
        self._pos_y        = 0.0
        self._yaw          = 0.0
        self._buggy_state  = 'WAITING_FOR_DESTINATION'
        self._odom_ready   = False

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)
        self.get_logger().info('WaypointFollowerNode started.')

    def _path_callback(self, msg: Path):
        self._waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._wp_index = 0
        self.get_logger().info(f'New path received: {len(self._waypoints)} waypoints.')

    def _odom_callback(self, msg: Odometry):
        self._pos_x    = msg.pose.pose.position.x
        self._pos_y    = msg.pose.pose.position.y
        self._yaw      = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_ready = True

    def _state_callback(self, msg: String):
        self._buggy_state = msg.data

    def _control_loop(self):
        # Only move when state machine says NAVIGATING
        if self._buggy_state != 'NAVIGATING':
            self._cmd_pub.publish(Twist())  # zero velocity
            return

        if not self._odom_ready or not self._waypoints:
            return

        if self._wp_index >= len(self._waypoints):
            return

        target_x, target_y = self._waypoints[self._wp_index]
        dx = target_x - self._pos_x
        dy = target_y - self._pos_y
        distance = math.hypot(dx, dy)

        # Arrival check
        if distance < ARRIVAL_RADIUS:
            self.get_logger().info(f'Reached waypoint {self._wp_index}')
            self._wp_index += 1

            if self._wp_index >= len(self._waypoints):
                self.get_logger().info('Final waypoint reached.')
                self._cmd_pub.publish(Twist())
                msg = String()
                msg.data = 'DESTINATION_REACHED'
                self._nav_pub.publish(msg)
                self._waypoints = []
            return

        # P-controller
        desired_heading = math.atan2(dy, dx)
        heading_error   = desired_heading - self._yaw

        # Normalise to [-π, π]
        while heading_error >  math.pi: heading_error -= 2.0 * math.pi
        while heading_error < -math.pi: heading_error += 2.0 * math.pi

        if abs(heading_error) < HEADING_DEADBAND:
            heading_error = 0.0

        angular_z = max(-MAX_ANGULAR_SPEED,
                        min(MAX_ANGULAR_SPEED, ANGULAR_P_GAIN * heading_error))

        if abs(heading_error) > HEADING_THRESHOLD:
            linear_x = SLOW_TURN_SPEED
        else:
            linear_x = min(MAX_LINEAR_SPEED, max(MIN_LINEAR_SPEED, distance * 1.5))

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
