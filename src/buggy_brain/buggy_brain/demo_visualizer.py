import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Point
import math

from buggy_brain.map_graph import NODES, EDGES


class DemoVisualizer(Node):
    """
    ROS 2 node that publishes RViz visualization markers for demo.

    Publishes:
        /visualization_markers  (visualization_msgs/MarkerArray)
            - Green spheres      : campus node locations
            - White text labels  : node names
            - Yellow line strip  : planned path
            - Red sphere         : obstacle location
            - Orange cylinder    : safety detection zone

    Subscribes:
        /planned_path    (nav_msgs/Path)
        /buggy_state     (std_msgs/String)
        /obstacle_info   (std_msgs/String)
        /crowd_info      (std_msgs/String)
    """

    def __init__(self):
        super().__init__('demo_visualizer')

        self.current_path     = []
        self.obstacle_x       = None
        self.obstacle_y       = None
        self.current_state    = 'WAITING_FOR_DESTINATION'

        # --- Subscribers ---
        self.create_subscription(
            Path, '/planned_path',
            self.path_callback, 10)
        self.create_subscription(
            String, '/buggy_state',
            self.state_callback, 10)
        self.create_subscription(
            String, '/obstacle_info',
            self.obstacle_callback, 10)

        # --- Publisher ---
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_markers', 10)

        # --- Publish markers at 2 Hz ---
        self.create_timer(0.5, self.publish_markers)

        self.get_logger().info('Demo visualizer started.')

    def path_callback(self, msg):
        self.current_path = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]

    def state_callback(self, msg):
        self.current_state = msg.data

    def obstacle_callback(self, msg):
        # Parse "DETECTED: obstacle at 1.23m" — we don't have
        # exact xy from lidar alone so we mark buggy front
        pass

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id    = 0

        # --- 1. Green spheres for each campus node ---
        for node_name, (x, y) in NODES.items():
            sphere          = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp    = self.get_clock().now().to_msg()
            sphere.ns       = 'campus_nodes'
            sphere.id       = marker_id
            sphere.type     = Marker.SPHERE
            sphere.action   = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.5
            sphere.pose.orientation.w = 1.0
            sphere.scale.x  = 3.0
            sphere.scale.y  = 3.0
            sphere.scale.z  = 3.0
            sphere.color.r  = 0.0
            sphere.color.g  = 1.0
            sphere.color.b  = 0.0
            sphere.color.a  = 0.8
            marker_array.markers.append(sphere)
            marker_id += 1

        # --- 2. White text labels for each node ---
        for node_name, (x, y) in NODES.items():
            label           = Marker()
            label.header.frame_id = 'map'
            label.header.stamp    = self.get_clock().now().to_msg()
            label.ns        = 'node_labels'
            label.id        = marker_id
            label.type      = Marker.TEXT_VIEW_FACING
            label.action    = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 2.5
            label.pose.orientation.w = 1.0
            label.scale.z   = 2.0
            label.color.r   = 1.0
            label.color.g   = 1.0
            label.color.b   = 1.0
            label.color.a   = 1.0
            label.text      = node_name.replace('_', '\n')
            marker_array.markers.append(label)
            marker_id += 1

        # --- 3. Grey lines for road edges ---
        for node_name, neighbours in EDGES.items():
            x1, y1 = NODES[node_name]
            for neighbour, _ in neighbours:
                x2, y2 = NODES[neighbour]
                edge            = Marker()
                edge.header.frame_id = 'map'
                edge.header.stamp    = self.get_clock().now().to_msg()
                edge.ns         = 'road_edges'
                edge.id         = marker_id
                edge.type       = Marker.LINE_STRIP
                edge.action     = Marker.ADD
                edge.scale.x    = 0.5
                edge.color.r    = 0.6
                edge.color.g    = 0.6
                edge.color.b    = 0.6
                edge.color.a    = 0.5
                edge.pose.orientation.w = 1.0
                p1       = Point()
                p1.x, p1.y, p1.z = x1, y1, 0.1
                p2       = Point()
                p2.x, p2.y, p2.z = x2, y2, 0.1
                edge.points = [p1, p2]
                marker_array.markers.append(edge)
                marker_id += 1

        # --- 4. Yellow line strip for planned path ---
        if len(self.current_path) >= 2:
            path_line           = Marker()
            path_line.header.frame_id = 'map'
            path_line.header.stamp    = self.get_clock().now().to_msg()
            path_line.ns        = 'planned_path'
            path_line.id        = marker_id
            path_line.type      = Marker.LINE_STRIP
            path_line.action    = Marker.ADD
            path_line.scale.x   = 1.0
            path_line.color.r   = 1.0
            path_line.color.g   = 1.0
            path_line.color.b   = 0.0
            path_line.color.a   = 1.0
            path_line.pose.orientation.w = 1.0
            for x, y in self.current_path:
                p     = Point()
                p.x, p.y, p.z = x, y, 0.3
                path_line.points.append(p)
            marker_array.markers.append(path_line)
            marker_id += 1

        # --- 5. State display text ---
        state_text           = Marker()
        state_text.header.frame_id = 'map'
        state_text.header.stamp    = self.get_clock().now().to_msg()
        state_text.ns        = 'buggy_state'
        state_text.id        = marker_id
        state_text.type      = Marker.TEXT_VIEW_FACING
        state_text.action    = Marker.ADD
        state_text.pose.position.x = 0.0
        state_text.pose.position.y = -45.0
        state_text.pose.position.z = 5.0
        state_text.pose.orientation.w = 1.0
        state_text.scale.z   = 4.0
        state_text.color.r   = 1.0
        state_text.color.g   = 0.8
        state_text.color.b   = 0.0
        state_text.color.a   = 1.0
        state_text.text      = f'STATE: {self.current_state}'
        marker_array.markers.append(state_text)
        marker_id += 1

        # --- 6. Safety detection ring around origin ---
        ring           = Marker()
        ring.header.frame_id = 'map'
        ring.header.stamp    = self.get_clock().now().to_msg()
        ring.ns        = 'safety_ring'
        ring.id        = marker_id
        ring.type      = Marker.LINE_STRIP
        ring.action    = Marker.ADD
        ring.scale.x   = 0.3
        ring.pose.orientation.w = 1.0

        # Color changes based on state
        if self.current_state == 'EMERGENCY_STOP':
            ring.color.r = 1.0
            ring.color.g = 0.0
            ring.color.b = 0.0
        elif self.current_state == 'NAVIGATING':
            ring.color.r = 0.0
            ring.color.g = 1.0
            ring.color.b = 0.0
        else:
            ring.color.r = 1.0
            ring.color.g = 0.5
            ring.color.b = 0.0
        ring.color.a = 0.7

        # Draw circle with radius 5m
        steps = 36
        for i in range(steps + 1):
            angle = 2.0 * math.pi * i / steps
            p     = Point()
            p.x   = 5.0 * math.cos(angle)
            p.y   = 5.0 * math.sin(angle)
            p.z   = 0.2
            ring.points.append(p)
        marker_array.markers.append(ring)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DemoVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
