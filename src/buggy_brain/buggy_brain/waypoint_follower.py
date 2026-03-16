import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.goal_tolerance    = 4.0
        self.linear_speed      = 1.0
        self.angular_gain      = 0.7
        self.max_angular_speed = 1.0

        self.waypoints     = []
        self.current_wp    = 0
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.robot_yaw     = 0.0
        self.active        = False
        self.stopped       = False
        self.odom_received = False

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.safety_sub = self.create_subscription(
            String, '/safety_status', self.safety_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub  = self.create_publisher(String, '/waypoint_status', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Waypoint follower started.')

    def path_callback(self, msg):
        self.waypoints  = [(p.pose.position.x, p.pose.position.y)
                           for p in msg.poses]
        self.current_wp = 0
        self.active     = True
        self.stopped    = False
        self.get_logger().info(
            f'New path received — {len(self.waypoints)} waypoints.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def safety_callback(self, msg):
        if msg.data == 'STOP':
            if not self.stopped:
                self.get_logger().warn('Safety stop received — halting buggy.')
            self.stopped = True
        elif msg.data == 'CLEAR':
            if self.stopped:
                self.get_logger().info('Path clear — resuming navigation.')
            self.stopped = False

    def control_loop(self):
        cmd = Twist()

        if not self.active or self.stopped or not self.waypoints:
            self.cmd_vel_pub.publish(cmd)
            return

        if not self.odom_received:
            self.cmd_vel_pub.publish(cmd)
            return

        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('Mission complete.')
            self.active = False
            self.cmd_vel_pub.publish(cmd)
            self._publish_status('MISSION_COMPLETE')
            return

        tx, ty = self.waypoints[self.current_wp]
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f'WP{self.current_wp}({tx:.0f},{ty:.0f}) '
            f'pos({self.robot_x:.1f},{self.robot_y:.1f}) '
            f'dist={distance:.1f}m yaw={math.degrees(self.robot_yaw):.0f}deg')

        if distance < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.current_wp} reached!')
            self.current_wp += 1
            self._publish_status(f'WAYPOINT_{self.current_wp}_REACHED')
            return

        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.robot_yaw

        while angle_error >  math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        angular_vel = self.angular_gain * angle_error
        angular_vel = max(-self.max_angular_speed,
                          min(self.max_angular_speed, angular_vel))

        if abs(angle_error) > 1.5:
            linear_vel = 0.0
        elif abs(angle_error) > 0.3:
            linear_vel = self.linear_speed * 0.4
        else:
            linear_vel = self.linear_speed

        cmd.linear.x  = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    def _publish_status(self, text):
        msg      = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
