import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class CrowdDetector(Node):
    """
    ROS 2 node that detects crowd presence using LiDAR ray counting.
    Counts the number of valid LiDAR rays that hit objects within a
    defined range band — a high count indicates a crowd or dense obstacle.

    Subscribes : /scan            (sensor_msgs/LaserScan)
    Publishes  : /crowd_detected  (std_msgs/String)
                 /crowd_info      (std_msgs/String)
    """

    def __init__(self):
        super().__init__('crowd_detector')

        # --- Parameters ---
        self.min_crowd_range    = 0.5    # metres — ignore closer than this
        self.max_crowd_range    = 2.5    # metres — ignore farther than this
        self.crowd_threshold    = 80     # ray count above this = crowd
        self.clear_threshold    = 60     # ray count below this = clear
        self.front_arc_deg      = 120.0  # degrees — detection arc width

        # --- State ---
        self.crowd_present = False

        # --- Subscriber ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10)

        # --- Publishers ---
        self.crowd_pub = self.create_publisher(
            String, '/crowd_detected', 10)
        self.info_pub  = self.create_publisher(
            String, '/crowd_info', 10)

        self.get_logger().info('Crowd detector started.')
        self.get_logger().info(
            f'Range band: {self.min_crowd_range}m - {self.max_crowd_range}m | '
            f'Threshold: {self.crowd_threshold} rays | '
            f'Arc: ±{self.front_arc_deg/2:.0f}°'
        )

    def scan_callback(self, msg):
        """Count LiDAR rays in the crowd detection band."""
        ray_count = self._count_crowd_rays(msg)

        if not self.crowd_present:
            if ray_count > self.crowd_threshold:
                self.crowd_present = True
                self.get_logger().warn(
                    f'CROWD DETECTED — {ray_count} rays in band')
                self._publish_crowd('CROWD_DETECTED')
                self._publish_info(
                    f'CROWD: {ray_count} rays detected in {self.min_crowd_range}-{self.max_crowd_range}m band')
        else:
            if ray_count < self.clear_threshold:
                self.crowd_present = False
                self.get_logger().info(
                    f'Crowd cleared — {ray_count} rays in band')
                self._publish_crowd('CROWD_CLEARED')
                self._publish_info(f'CLEAR: {ray_count} rays')
            else:
                # Still crowded — keep publishing
                self._publish_crowd('CROWD_DETECTED')

    def _count_crowd_rays(self, msg):
        """Count valid rays within the front arc and crowd range band."""
        half_arc_rad = math.radians(self.front_arc_deg / 2.0)
        angle        = msg.angle_min
        count        = 0

        for r in msg.ranges:
            if -half_arc_rad <= angle <= half_arc_rad:
                if (not math.isnan(r) and
                        not math.isinf(r) and
                        self.min_crowd_range <= r <= self.max_crowd_range):
                    count += 1
            angle += msg.angle_increment

        return count

    def _publish_crowd(self, status):
        msg      = String()
        msg.data = status
        self.crowd_pub.publish(msg)

    def _publish_info(self, text):
        msg      = String()
        msg.data = text
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CrowdDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
