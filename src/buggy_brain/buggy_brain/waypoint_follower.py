#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32

# Tuning constants
MAX_LINEAR_SPEED  = 1.00
MIN_LINEAR_SPEED  = 0.30
ANGULAR_P_GAIN    = 1.00
MAX_ANGULAR_SPEED = 0.80
ARRIVAL_RADIUS    = 5.00
SLOW_TURN_SPEED   = 0.20
HEADING_THRESHOLD = 0.30
HEADING_DEADBAND  = 0.05
TURN_IN_PLACE_THR = 0.80
CONTROL_RATE_HZ   = 20

def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollowerNode(Node):

    def __init__(self):
        super().__init__('waypoint_follower_node')

        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',           10)
        self._nav_pub    = self.create_publisher(String, '/navigation_command', 10)
        self._status_pub = self.create_publisher(String, '/waypoint_status',    10)

        self.create_subscription(Path,    '/planned_path',      self._path_callback,     10)
        self.create_subscription(Odometry,'/odom',              self._odom_callback,     10)
        self.create_subscription(String,  '/buggy_state',       self._state_callback,    10)
        self.create_subscription(String,  '/safety_status',     self._safety_callback,   10)
        self.create_subscription(Float32, '/avoidance_steering',self._avoidance_callback,10)

        self._waypoints        = []
        self._wp_index         = 0
        self._pos_x            = 0.0
        self._pos_y            = 0.0
        self._yaw              = 0.0
        self._buggy_state      = 'WAITING_FOR_DESTINATION'
        self._odom_ready       = False
        self._avoidance_offset = 0.0
        self._stopped          = False

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)
        self.get_logger().info('WaypointFollowerNode started.')

    def _path_callback(self, msg: Path):
        self._waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._wp_index = 0
        self._stopped  = False
        self.get_logger().info(
            f'New path: {len(self._waypoints)} waypoints: {self._waypoints}')

    def _odom_callback(self, msg: Odometry):
        self._pos_x      = msg.pose.pose.position.x
        self._pos_y      = msg.pose.pose.position.y
        self._yaw        = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_ready = True

    def _state_callback(self, msg: String):
        self._buggy_state = msg.data

    def _safety_callback(self, msg: String):
        if msg.data == 'STOP':
            if not self._stopped:
                self.get_logger().warn('Safety STOP received.')
            self._stopped = True
        elif msg.data == 'CLEAR':
            if self._stopped:
                self.get_logger().info('Path CLEAR — resuming.')
            self._stopped = False

    def _avoidance_callback(self, msg: Float32):
        self._avoidance_offset = msg.data

    def _control_loop(self):
        cmd = Twist()

        if self._buggy_state != 'NAVIGATING' or self._stopped:
            self._cmd_pub.publish(cmd)
            return

        if not self._odom_ready or not self._waypoints:
            return

        if self._wp_index >= len(self._waypoints):
            return

        target_x, target_y = self._waypoints[self._wp_index]
        dx = target_x - self._pos_x
        dy = target_y - self._pos_y
        distance = math.hypot(dx, dy)

        self.get_logger().info(
            f'WP{self._wp_index}({target_x:.0f},{target_y:.0f}) '
            f'pos({self._pos_x:.1f},{self._pos_y:.1f}) '
            f'dist={distance:.1f}m yaw={math.degrees(self._yaw):.0f}deg',
            throttle_duration_sec=1.0
        )

        if distance < ARRIVAL_RADIUS:
            self.get_logger().info(f'Waypoint {self._wp_index} reached!')
            self._wp_index += 1
            self._publish_status(f'WAYPOINT_{self._wp_index}_REACHED')

            if self._wp_index >= len(self._waypoints):
                self.get_logger().info('Final destination reached!')
                self._cmd_pub.publish(Twist())
                nav_msg      = String()
                nav_msg.data = 'DESTINATION_REACHED'
                self._nav_pub.publish(nav_msg)
                self._publish_status('MISSION_COMPLETE')
                self._waypoints = []
            return

        desired_heading = math.atan2(dy, dx)
        heading_error   = desired_heading - self._yaw

        while heading_error >  math.pi: heading_error -= 2.0 * math.pi
        while heading_error < -math.pi: heading_error += 2.0 * math.pi

        if abs(heading_error) < HEADING_DEADBAND:
            heading_error = 0.0

        angular_z = max(-MAX_ANGULAR_SPEED,
                        min(MAX_ANGULAR_SPEED, ANGULAR_P_GAIN * heading_error))

        if abs(heading_error) > TURN_IN_PLACE_THR:
            linear_x = 0.0
        elif abs(heading_error) > HEADING_THRESHOLD:
            linear_x = SLOW_TURN_SPEED
        else:
            linear_x = min(MAX_LINEAR_SPEED,
                           max(MIN_LINEAR_SPEED, distance * 0.5))

        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z + self._avoidance_offset
        self._cmd_pub.publish(cmd)

    def _publish_status(self, text):
        msg      = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f'Shutdown warning: {e}')


if __name__ == '__main__':
    main()
