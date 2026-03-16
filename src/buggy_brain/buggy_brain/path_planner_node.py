import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from buggy_brain.map_graph import find_shortest_path, get_path_coordinates, EDGES


class PathPlannerNode(Node):
    """
    ROS 2 node that accepts a destination name and publishes
    a planned path as a nav_msgs/Path message.

    Subscribes : /goal_destination  (std_msgs/String)
    Publishes  : /planned_path      (nav_msgs/Path)
                 /planning_status   (std_msgs/String)
    """

    def __init__(self):
        super().__init__('path_planner_node')

        # Current start position — in a real system this comes from GPS/localisation
        # For SITL we default to MAIN_GATE
        self.current_position = 'MAIN_GATE'

        # Subscriber — listens for destination commands
        self.goal_sub = self.create_subscription(
            String,
            '/goal_destination',
            self.goal_callback,
            10
        )

        # Publisher — sends the planned path to waypoint_follower
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Publisher — sends human-readable status strings for debugging
        self.status_pub = self.create_publisher(
            String,
            '/planning_status',
            10
        )

        self.get_logger().info('Path planner node started.')
        self.get_logger().info(f'Current position: {self.current_position}')
        self.get_logger().info('Waiting for goal on /goal_destination ...')

    def goal_callback(self, msg):
        """Called every time a new destination is published to /goal_destination."""
        destination = msg.data.strip().upper()

        self.get_logger().info(f'Received goal: {destination}')

        # Validate destination
        if destination not in EDGES:
            warning = f'Unknown destination: {destination}. Valid: {list(EDGES.keys())}'
            self.get_logger().warn(warning)
            self._publish_status(f'ERROR: {warning}')
            return

        if destination == self.current_position:
            self.get_logger().info('Already at destination.')
            self._publish_status(f'Already at {destination}')
            return

        # Compute shortest path
        path_nodes = find_shortest_path(EDGES, self.current_position, destination)

        if not path_nodes:
            self.get_logger().error(f'No path found from {self.current_position} to {destination}')
            self._publish_status(f'ERROR: No path from {self.current_position} to {destination}')
            return

        self.get_logger().info(f'Path found: {" -> ".join(path_nodes)}')

        # Convert node names to (x, y) coordinates
        coordinates = get_path_coordinates(path_nodes)

        # Build and publish nav_msgs/Path
        path_msg = self._build_path_message(coordinates)
        self.path_pub.publish(path_msg)

        # Publish status
        self._publish_status(f'PLANNING: {" -> ".join(path_nodes)}')

        # Update current position to destination
        self.current_position = destination
        self.get_logger().info(f'Path published to /planned_path ({len(coordinates)} waypoints)')

    def _build_path_message(self, coordinates):
        """Converts a list of (x, y) tuples into a nav_msgs/Path message."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Facing forward, no rotation
            path_msg.poses.append(pose)

        return path_msg

    def _publish_status(self, text):
        """Publishes a status string to /planning_status."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


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
