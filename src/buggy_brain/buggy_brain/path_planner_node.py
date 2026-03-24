import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from buggy_brain.map_graph import find_shortest_path, get_path_coordinates, EDGES


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        self.current_position = 'SRM_IST'

        self.goal_sub = self.create_subscription(
            String, '/goal_destination', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.status_pub = self.create_publisher(String, '/planning_status', 10)

        self.get_logger().info('Path planner node started.')
        self.get_logger().info(f'Current position: {self.current_position}')
        self.get_logger().info('Waiting for goal on /goal_destination ...')

    def goal_callback(self, msg):
        destination = msg.data.strip().upper()
        self.get_logger().info(f'Received goal: {destination}')

        if destination not in EDGES:
            warning = f'Unknown destination: {destination}. Valid: {list(EDGES.keys())}'
            self.get_logger().warn(warning)
            self._publish_status(f'ERROR: {warning}')
            return

        if destination == self.current_position:
            self.get_logger().info('Already at destination.')
            self._publish_status(f'Already at {destination}')
            return

        path_nodes = find_shortest_path(self.current_position, destination)

        if not path_nodes:
            self.get_logger().error(
                f'No path found from {self.current_position} to {destination}')
            self._publish_status(
                f'ERROR: No path from {self.current_position} to {destination}')
            return

        self.get_logger().info(f'Path found: {" -> ".join(path_nodes)}')

        coordinates = get_path_coordinates(path_nodes)
        path_msg = self._build_path_message(coordinates)
        self.path_pub.publish(path_msg)

        self._publish_status(f'PLANNING: {" -> ".join(path_nodes)}')
        self.current_position = destination
        self.get_logger().info(
            f'Path published to /planned_path ({len(coordinates)} waypoints)')

    def _build_path_message(self, coordinates):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg

    def _publish_status(self, text):
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
