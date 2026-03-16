import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class ObstacleDetector(Node):
    """
    ROS 2 node that monitors LiDAR scan data and publishes
    safety stop/clear commands based on obstacle proximity.

    Subscribes : /scan           (sensor_msgs/LaserScan)
    Publishes  : /safety_status  (std_msgs/String)
                 /obstacle_info  (std_msgs/String)
    """

    def __init__(self):
        super().__init__('obstacle_detector')

        # --- Parameters ---
        # Distance thresholds in metres
        self.stop_distance   = 2.0   # stop if obstacle closer than this
        self.clear_distance  = 3.0   # resume only when obstacle beyond this
        self.front_arc_deg   = 60.0  # degrees — only check front-facing arc

        # --- State ---
        self.obstacle_present = False  # True when currently stopped for obstacle

        # --- Subscriber ---
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # --- Publishers ---
        self.safety_pub   = self.create_publisher(String, '/safety_status', 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle_info', 10)

        self.get_logger().info('Obstacle detector started.')
        self.get_logger().info(
            f'Stop threshold: {self.stop_distance}m | '
            f'Clear threshold: {self.clear_distance}m | '
            f'Front arc: ±{self.front_arc_deg/2:.0f}°'
        )

    def scan_callback(self, msg):
        """Process each LiDAR scan and publish safety commands."""
        min_distance = self._get_min_front_distance(msg)

        if min_distance is None:
            return

        if not self.obstacle_present:
            # Currently moving — check if we need to stop
            if min_distance < self.stop_distance:
                self.obstacle_present = True
                self.get_logger().warn(
                    f'OBSTACLE DETECTED at {min_distance:.2f}m — sending STOP')
                self._publish_safety('STOP')
                self._publish_obstacle_info(min_distance, 'DETECTED')
        else:
            # Currently stopped — check if path is clear to resume
            if min_distance > self.clear_distance:
                self.obstacle_present = False
                self.get_logger().info(
                    f'Path clear at {min_distance:.2f}m — sending CLEAR')
                self._publish_safety('CLEAR')
                self._publish_obstacle_info(min_distance, 'CLEARED')
            else:
                # Still blocked — keep publishing STOP at each scan
                self._publish_safety('STOP')

    def _get_min_front_distance(self, msg):
        """
        Returns the minimum valid range reading within the front arc.
        Filters out inf, nan, and out-of-range values.
        """
        half_arc_rad = math.radians(self.front_arc_deg / 2.0)

        angle      = msg.angle_min
        min_dist   = float('inf')
        valid_count = 0

        for r in msg.ranges:
            # Only consider readings within the front-facing arc
            if -half_arc_rad <= angle <= half_arc_rad:
                if (not math.isnan(r) and
                        not math.isinf(r) and
                        msg.range_min <= r <= msg.range_max):
                    min_dist    = min(min_dist, r)
                    valid_count += 1

            angle += msg.angle_increment

        if valid_count == 0:
            return None

        return min_dist

    def _publish_safety(self, status):
        """Publish STOP or CLEAR to /safety_status."""
        msg      = String()
        msg.data = status
        self.safety_pub.publish(msg)

    def _publish_obstacle_info(self, distance, event):
        """Publish human-readable obstacle info for debugging."""
        msg      = String()
        msg.data = f'{event}: obstacle at {distance:.2f}m'
        self.obstacle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
